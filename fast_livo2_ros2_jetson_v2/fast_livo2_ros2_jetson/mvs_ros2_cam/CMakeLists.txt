cmake_minimum_required(VERSION 3.10)
project(mvs_ros2_cam)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-O3)

# ---------- MVS SDK ----------
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(MVS_LIB_DIR /opt/MVS/lib/aarch64)
  message(STATUS "MVS: aarch64")
else()
  set(MVS_LIB_DIR /opt/MVS/lib/64)
  message(STATUS "MVS: x86_64")
endif()

# ---------- ROS2 deps ----------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  /opt/MVS/include
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${MVS_LIB_DIR})

# ---------- executable ----------
add_executable(grab_trigger src/grab_trigger_ros2.cpp)

ament_target_dependencies(grab_trigger
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
)

target_link_libraries(grab_trigger
  MvCameraControl
  ${OpenCV_LIBS}
  pthread
)

# ---------- install ----------
install(TARGETS grab_trigger
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
