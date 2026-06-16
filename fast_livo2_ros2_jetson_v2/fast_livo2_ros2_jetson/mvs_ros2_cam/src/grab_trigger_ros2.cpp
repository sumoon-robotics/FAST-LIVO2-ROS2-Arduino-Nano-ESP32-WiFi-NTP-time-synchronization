/**
 * grab_trigger_ros2.cpp
 * ROS2 port of LIV_handhold_2/mvs_ros_driver/src/grab_trigger.cpp
 *
 * Preserves:
 *   - Shared-memory timestamp from ~/timeshare (STM32 sync)
 *   - Hardware trigger on Line0
 *   - All camera params via OpenCV FileStorage YAML (argv[1])
 *   - Serial-number-based camera selection
 *   - MV_CC_ConvertPixelType pixel conversion pipeline
 */

#include "MvCameraControl.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <pthread.h>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <signal.h>

using namespace std;

// -------------------------------------------------------
// Shared-memory timestamp struct (matches STM32 writer)
// -------------------------------------------------------
struct time_stamp {
  int64_t high;
  int64_t low;   // nanoseconds since epoch (written by timesync bridge)
};
static time_stamp *pointt = nullptr;

// -------------------------------------------------------
// Globals
// -------------------------------------------------------
enum PixelFormat : unsigned int {
  RGB8             = 0x02180014,
  BayerRG8         = 0x01080009,
  BayerRG12Packed  = 0x010C002B,
  BayerGB12Packed  = 0x010C002C,
  BayerGB8         = 0x0108000A
};

static bool exit_flag      = false;
static int  trigger_enable = 1;
static float image_scale   = 1.0f;
static int  width  = 0;
static int  height = 0;

static std::vector<PixelFormat> PIXEL_FORMAT =
  { RGB8, BayerRG8, BayerRG12Packed, BayerGB12Packed, BayerGB8 };

static const std::string ExposureAutoStr[3] = {"Off", "Once", "Continues"};
static const std::string GammaSlectorStr[3] = {"User", "sRGB", "Off"};
static const std::string GainAutoStr[3]     = {"Off", "Once", "Continues"};

// Publisher — set before WorkThread starts
static rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr g_pub;
static rclcpp::Node::SharedPtr                               g_node;

// -------------------------------------------------------
// Signal handler
// -------------------------------------------------------
static void SignalHandler(int sig) {
  if (sig == SIGINT) {
    fprintf(stderr, "\nCaught SIGINT, shutting down...\n");
    exit_flag = true;
    rclcpp::shutdown();
  }
}

// -------------------------------------------------------
// Print device info
// -------------------------------------------------------
static bool PrintDeviceInfo(MV_CC_DEVICE_INFO *info) {
  if (!info) return false;
  if (info->nTLayerType == MV_USB_DEVICE) {
    printf("  Model: %s  SN: %s\n",
      info->SpecialInfo.stUsb3VInfo.chModelName,
      info->SpecialInfo.stUsb3VInfo.chSerialNumber);
  } else if (info->nTLayerType == MV_GIGE_DEVICE) {
    printf("  Model: %s  SN: %s\n",
      info->SpecialInfo.stGigEInfo.chModelName,
      info->SpecialInfo.stGigEInfo.chSerialNumber);
  }
  return true;
}

// -------------------------------------------------------
// Set camera params from OpenCV YAML
// -------------------------------------------------------
static void setParams(void *handle, const std::string &params_file) {
  cv::FileStorage fs(params_file, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    RCLCPP_ERROR(g_node->get_logger(),
      "Failed to open params file: %s", params_file.c_str());
    return;
  }

  image_scale = (float)fs["image_scale"];
  if (image_scale < 0.1f) image_scale = 1.0f;

  int ExposureAutoMode   = (int)fs["ExposureAutoMode"];
  int ExposureTimeLower  = (int)fs["AutoExposureTimeLower"];
  int ExposureTimeUpper  = (int)fs["AutoExposureTimeUpper"];
  int ExposureTime       = (int)fs["ExposureTime"];
  int GainAuto           = (int)fs["GainAuto"];
  float Gain             = (float)fs["Gain"];
  float Gamma            = (float)fs["Gamma"];
  int GammaSlector       = (int)fs["GammaSelector"];

  int nRet;

  // Exposure mode
  nRet = MV_CC_SetExposureAutoMode(handle, ExposureAutoMode);
  RCLCPP_INFO(g_node->get_logger(), "ExposureAutoMode → %s (ret=0x%x)",
    ExposureAutoStr[ExposureAutoMode].c_str(), nRet);

  if (ExposureAutoMode == 2) {  // Continuous auto
    MV_CC_SetAutoExposureTimeLower(handle, ExposureTimeLower);
    MV_CC_SetAutoExposureTimeUpper(handle, ExposureTimeUpper);
    RCLCPP_INFO(g_node->get_logger(),
      "Auto exposure range: [%d, %d] us", ExposureTimeLower, ExposureTimeUpper);
  } else if (ExposureAutoMode == 0) {  // Fixed
    nRet = MV_CC_SetExposureTime(handle, ExposureTime);
    RCLCPP_INFO(g_node->get_logger(),
      "Fixed ExposureTime: %d us (ret=0x%x)", ExposureTime, nRet);
  }

  // Gain
  nRet = MV_CC_SetEnumValue(handle, "GainAuto", GainAuto);
  RCLCPP_INFO(g_node->get_logger(), "GainAuto → %s (ret=0x%x)",
    GainAutoStr[GainAuto].c_str(), nRet);
  if (GainAuto == 0) {
    MV_CC_SetGain(handle, Gain);
    RCLCPP_INFO(g_node->get_logger(), "Gain → %.2f", Gain);
  }

  // Gamma
  nRet = MV_CC_SetGammaSelector(handle, GammaSlector);
  RCLCPP_INFO(g_node->get_logger(), "GammaSelector → %s (ret=0x%x)",
    GammaSlectorStr[GammaSlector].c_str(), nRet);
  MV_CC_SetGamma(handle, Gamma);
  RCLCPP_INFO(g_node->get_logger(), "Gamma → %.2f", Gamma);
}

// -------------------------------------------------------
// Worker grab thread — mirrors original logic
// -------------------------------------------------------
static void *WorkThread(void *pUser) {
  int nRet;
  MVCC_INTVALUE stParam;
  memset(&stParam, 0, sizeof(MVCC_INTVALUE));

  nRet = MV_CC_GetIntValue(pUser, "PayloadSize", &stParam);
  if (MV_OK != nRet) {
    printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
    return nullptr;
  }

  MV_FRAME_OUT_INFO_EX      stImageInfo = {0};
  MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};

  unsigned char *pData       = (unsigned char *)malloc(stParam.nCurValue * 3);
  unsigned char *pDataForBGR = (unsigned char *)malloc(stParam.nCurValue * 3);

  if (!pData || !pDataForBGR) {
    RCLCPP_FATAL(g_node->get_logger(), "Frame buffer malloc failed");
    if (pData)       free(pData);
    if (pDataForBGR) free(pDataForBGR);
    return nullptr;
  }

  while (!exit_flag && rclcpp::ok()) {
    nRet = MV_CC_GetOneFrameTimeout(pUser, pData,
                                    stParam.nCurValue * 3,
                                    &stImageInfo, 1000);
    if (nRet != MV_OK) continue;

    // ---------------------------------------------------
    // Timestamp: prefer STM32 shared-memory, else system
    // ---------------------------------------------------
    rclcpp::Time stamp;
    if (trigger_enable && pointt && pointt != MAP_FAILED && pointt->low != 0) {
      int64_t ns = pointt->low;
      stamp = rclcpp::Time(ns, RCL_ROS_TIME);
    } else {
      stamp = g_node->now();
    }

    // ---------------------------------------------------
    // Pixel conversion → RGB8
    // ---------------------------------------------------
    stConvertParam.nWidth          = stImageInfo.nWidth;
    stConvertParam.nHeight         = stImageInfo.nHeight;
    stConvertParam.pSrcData        = pData;
    stConvertParam.nSrcDataLen     = stParam.nCurValue * 3;
    stConvertParam.enSrcPixelType  = stImageInfo.enPixelType;
    stConvertParam.enDstPixelType  = PixelType_Gvsp_RGB8_Packed;
    stConvertParam.pDstBuffer      = pDataForBGR;
    stConvertParam.nDstBufferSize  = stParam.nCurValue * 3;

    nRet = MV_CC_ConvertPixelType(pUser, &stConvertParam);
    if (MV_OK != nRet) {
      printf("ConvertPixelType failed 0x%x, skipping frame\n", nRet);
      continue;
    }

    cv::Mat srcImage(stImageInfo.nHeight, stImageInfo.nWidth,
                     CV_8UC3, pDataForBGR);

    if (image_scale > 0.1f && image_scale != 1.0f) {
      cv::resize(srcImage, srcImage,
        cv::Size((int)(srcImage.cols * image_scale),
                 (int)(srcImage.rows * image_scale)),
        0, 0, cv::INTER_LINEAR);
    }

    // ---------------------------------------------------
    // Publish
    // ---------------------------------------------------
    auto msg = cv_bridge::CvImage(
      std_msgs::msg::Header(), "rgb8", srcImage).toImageMsg();
    msg->header.stamp    = stamp;
    msg->header.frame_id = "camera";

    g_pub->publish(*msg);
  }

  free(pData);
  free(pDataForBGR);
  return nullptr;
}

// -------------------------------------------------------
// main
// -------------------------------------------------------
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  g_node = rclcpp::Node::make_shared("mvs_trigger");

  if (argc < 2) {
    RCLCPP_FATAL(g_node->get_logger(),
      "Usage: grab_trigger <path_to_cam_config.yaml>");
    return -1;
  }
  std::string params_file = argv[1];

  // ---- Read top-level config ----
  cv::FileStorage fs(params_file, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    RCLCPP_FATAL(g_node->get_logger(),
      "Cannot open config: %s", params_file.c_str());
    return -1;
  }
  trigger_enable = (int)fs["TriggerEnable"];
  std::string expect_sn  = (std::string)fs["SerialNumber"];
  std::string pub_topic  = (std::string)fs["TopicName"];
  int pixel_format_idx   = (int)fs["PixelFormat"];
  fs.release();

  RCLCPP_INFO(g_node->get_logger(),
    "Config: trigger=%d  topic=%s  pixfmt=%d  sn='%s'",
    trigger_enable, pub_topic.c_str(), pixel_format_idx, expect_sn.c_str());

  // ---- Publisher ----
  g_pub = g_node->create_publisher<sensor_msgs::msg::Image>(pub_topic, 10);

  // ---- Open shared-memory timestamp file (~/timeshare) ----
  const char *user_name = getlogin();
  std::string timeshare_path = std::string("/home/") + user_name + "/timeshare";
  int fd = open(timeshare_path.c_str(), O_RDWR);
  if (fd < 0) {
    RCLCPP_WARN(g_node->get_logger(),
      "~/timeshare not found — using system clock for timestamps");
    pointt = nullptr;
  } else {
    pointt = (time_stamp *)mmap(nullptr, sizeof(time_stamp),
                                PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (pointt == MAP_FAILED) {
      RCLCPP_WARN(g_node->get_logger(), "mmap failed — using system clock");
      pointt = nullptr;
    } else {
      RCLCPP_INFO(g_node->get_logger(),
        "STM32 shared-memory timestamp active (%s)", timeshare_path.c_str());
    }
  }

  // ---- Signal handler ----
  struct sigaction sa{};
  sa.sa_handler = SignalHandler;
  sigemptyset(&sa.sa_mask);
  sigaction(SIGINT, &sa, nullptr);

  // ---- Enumerate cameras ----
  MV_CC_DEVICE_INFO_LIST devList;
  memset(&devList, 0, sizeof(devList));
  int nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &devList);
  if (MV_OK != nRet || devList.nDeviceNum == 0) {
    RCLCPP_FATAL(g_node->get_logger(), "No HIKrobot cameras found");
    return -1;
  }

  for (unsigned i = 0; i < devList.nDeviceNum; i++) {
    printf("[device %u]: ", i);
    PrintDeviceInfo(devList.pDeviceInfo[i]);
  }

  // ---- Select camera by serial number ----
  unsigned int nIndex = 0;
  if (devList.nDeviceNum > 1) {
    if (expect_sn.empty()) {
      RCLCPP_FATAL(g_node->get_logger(),
        "Multiple cameras found — set SerialNumber in config YAML");
      return -1;
    }
    bool found = false;
    for (unsigned i = 0; i < devList.nDeviceNum; i++) {
      auto *info = devList.pDeviceInfo[i];
      std::string sn;
      if (info->nTLayerType == MV_USB_DEVICE)
        sn = (char *)info->SpecialInfo.stUsb3VInfo.chSerialNumber;
      else if (info->nTLayerType == MV_GIGE_DEVICE)
        sn = (char *)info->SpecialInfo.stGigEInfo.chSerialNumber;
      if (sn == expect_sn) { nIndex = i; found = true; break; }
    }
    if (!found) {
      RCLCPP_FATAL(g_node->get_logger(),
        "Camera SN '%s' not found", expect_sn.c_str());
      return -1;
    }
  }

  // ---- Create handle & open ----
  void *handle = nullptr;
  MV_CC_CreateHandle(&handle, devList.pDeviceInfo[nIndex]);
  nRet = MV_CC_OpenDevice(handle);
  if (MV_OK != nRet) {
    RCLCPP_FATAL(g_node->get_logger(), "OpenDevice fail 0x%x", nRet);
    return -1;
  }

  // Disable free-run frame rate (trigger mode controls rate)
  MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", false);

  // Pixel format
  nRet = MV_CC_SetEnumValue(handle, "PixelFormat",
    PIXEL_FORMAT[pixel_format_idx]);
  if (MV_OK != nRet) {
    RCLCPP_FATAL(g_node->get_logger(),
      "SetPixelFormat fail 0x%x", nRet);
    return -1;
  }

  // Camera params (exposure, gain, gamma)
  setParams(handle, params_file);

  // Hardware trigger
  nRet = MV_CC_SetEnumValue(handle, "TriggerMode", trigger_enable);
  if (MV_OK != nRet) {
    RCLCPP_FATAL(g_node->get_logger(), "SetTriggerMode fail 0x%x", nRet);
    return -1;
  }
  nRet = MV_CC_SetEnumValue(handle, "TriggerSource",
    MV_TRIGGER_SOURCE_LINE0);
  if (MV_OK != nRet) {
    RCLCPP_FATAL(g_node->get_logger(), "SetTriggerSource fail 0x%x", nRet);
    return -1;
  }

  // Start grabbing
  nRet = MV_CC_StartGrabbing(handle);
  if (MV_OK != nRet) {
    RCLCPP_FATAL(g_node->get_logger(), "StartGrabbing fail 0x%x", nRet);
    return -1;
  }
  RCLCPP_INFO(g_node->get_logger(), "HIKrobot grabbing — topic: %s",
    pub_topic.c_str());

  // Worker thread
  pthread_t tid;
  pthread_create(&tid, nullptr, WorkThread, handle);

  // Spin ROS2
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(g_node);
  while (!exit_flag && rclcpp::ok()) {
    exec.spin_some(std::chrono::milliseconds(10));
  }

  pthread_join(tid, nullptr);

  MV_CC_StopGrabbing(handle);
  MV_CC_CloseDevice(handle);
  MV_CC_DestroyHandle(handle);

  if (pointt && pointt != MAP_FAILED)
    munmap(pointt, sizeof(time_stamp));

  rclcpp::shutdown();
  return 0;
}
