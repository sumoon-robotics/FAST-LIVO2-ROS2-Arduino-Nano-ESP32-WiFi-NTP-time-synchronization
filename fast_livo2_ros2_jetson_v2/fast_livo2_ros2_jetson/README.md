# FAST-LIVO2 ROS2 on Jetson AGX Orin

Native ROS2 Humble deployment of [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2) (LiDAR-Inertial-Visual Odometry) on NVIDIA Jetson AGX Orin with Livox Mid-360 LiDAR and HIKrobot MV-CA016-10UC camera. Fully working with real-time LIVO mapping.

---

## Hardware

| Component | Model | Interface |
|---|---|---|
| Compute | NVIDIA Jetson AGX Orin | — |
| LiDAR | Livox Mid-360 | Ethernet `192.168.1.100` |
| Camera | HIKrobot MV-CA016-10UC | USB3 |
| Time sync | Arduino Nano ESP32 | USB-C → Jetson; UART → Mid-360; GPIO → camera |

**Physical mount:** Camera below LiDAR, 3.75cm offset, forward-facing.

---

## Software Stack

| Package | Source | Notes |
|---|---|---|
| ROS2 Humble | `/opt/ros/humble` | Native (no Docker) |
| FAST-LIVO2 | `hku-mars/FAST-LIVO2` | Already ROS2-ported |
| livox_ros_driver2 | `~/ws_livox` | CustomMsg format, with GPRMC timesync JSON |
| rpg_vikit | `src/rpg_vikit` | Patched for aarch64 + Sophus |
| mvs_ros2_cam | `src/mvs_ros2_cam` | ROS2 port of LIV_handhold_2 driver |

**Workspace:** `/mnt/ssd/fast_livo2_ros2_ws`

---

## Timesync Architecture

```
Arduino Nano ESP32
│
├── WiFi NTP (pool.ntp.org) ──── auto-syncs on boot, no manual seeding
│
├── D1 TX1 (9600 baud GPRMC) ─► Mid-360 Gray/White
│                                 └── LiDAR clock = UTC (verified: sec delta = 0)
│
├── D4 TX2 (9600 baud GPRMC) ─► USB-TTL → /dev/ttyUSB0 (Jetson monitor)
│
├── D5 (PPS 1Hz) ─────────────► Mid-360 Purple/White
│
└── A0 (PWM 10Hz) ────────────► HIKrobot Line 2 trigger (+)
                                  └── camera captures in sync with trigger
```

**Result:** LiDAR clock = system clock = camera clock. No `img_time_offset` needed.

---

## Calibration

### Extrinsics (camera → LiDAR, hand-eye calibration)

```yaml
Rcl: [-0.002170, -0.999962,  0.008495,
       0.362654, -0.008704, -0.931883,
       0.931922,  0.001058,  0.362658]
Pcl: [0.034932, -0.040393, -0.037527]  # camera 3.75cm below LiDAR
```

### Camera Intrinsics (MV-CA016-10UC, calibrated)

```yaml
# Native 1440×1080, scale=0.5 → 720×540
cam_fx: 879.098   cam_fy: 879.804
cam_cx: 352.076   cam_cy: 272.032
cam_d0: -0.122045   cam_d1: 0.117873
cam_d2: -0.000884   cam_d3: -0.001858
```

---

## Build

```bash
cd /mnt/ssd/fast_livo2_ros2_ws
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash

# 1. vikit — must build first (FAST-LIVO2 hardcodes install/ path)
colcon build --packages-select vikit_common \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

colcon build --packages-select vikit_ros \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 2. HIKrobot camera driver
colcon build --packages-select mvs_ros2_cam \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# 3. FAST-LIVO2
colcon build --packages-select fast_livo \
  --packages-ignore livox_ros_driver2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Known Build Fixes Applied

| Package | Issue | Fix |
|---|---|---|
| `vikit_common` | `PROJECT()` before `cmake_minimum_required()` | Inject correct cmake line at top |
| `vikit_common` | `-mmmx -msse` x86 flags on aarch64 | Strip all x86 SIMD flags |
| `vikit_common` | catkin dependency | Rewrite CMakeLists for ament |
| `vikit_common` | `sophus/se3.hpp` not found | Add `Sophus::Sophus` via target property include |
| `vikit_ros` | `${cpp_typesupport_target}` undefined | Rewrite CMakeLists, link vikit_common via absolute path |
| `fast_livo` | ROS2 int/double type mismatch | All numeric values match `avia.yaml` reference types |

---

## Launch

### Startup script

```bash
~/start_fast_livo2.sh           # without RViz
~/start_fast_livo2.sh --rviz    # with RViz
```

### Manual (3 terminals)

**Terminal 1 — LiDAR:**
```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

**Terminal 2 — Camera:**
```bash
source /opt/ros/humble/setup.bash
source /mnt/ssd/fast_livo2_ros2_ws/install/setup.bash
ros2 launch mvs_ros2_cam mvs_camera.launch.py
```

**Terminal 3 — FAST-LIVO2:**
```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
source /mnt/ssd/fast_livo2_ros2_ws/install/setup.bash

# LD_PRELOAD required — fixes libpcl_io libusb conflict on Jetson
LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0 \
  ros2 launch fast_livo mapping_mid360_hikrobot.launch.py use_rviz:=True
```

### RViz2 setup

Fixed frame: `camera_init`

| Display | Topic |
|---|---|
| PointCloud2 | `/cloud_registered` |
| Path | `/path` |
| Odometry | `/Odometry` |
| Image | `/rgb_img` |

---

## Performance (Jetson AGX Orin)

| Stage | Time |
|---|---|
| LIO (ICP + voxel map) | ~30ms |
| VIO (Jacobian + EKF) | ~7ms |
| Effective LiDAR features | ~5300 per scan |
| Visual map points | 700+ sparse map |

---

## Arduino Nano ESP32 Firmware

See `firmware/livo2_timesync.ino`.

**Setup:**
1. Install Arduino IDE 2.x
2. Add board URL: `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
3. Install **esp32 by Espressif Systems**
4. Select **Tools → Board → Arduino Nano ESP32**
5. Edit `WIFI_SSID` and `WIFI_PASSWORD` at top of sketch
6. Upload — open Serial Monitor at 115200 baud

**Verify on Jetson:**
```bash
timeout 5 cat /dev/ttyUSB0
# Expected: $GPRMC,HHMMSS.000,A,0.0,N,0.0,E,0.0,0.0,DDMMYY,0.0,,A*XX
```

---

## Config Files

`/mnt/ssd/fast_livo2_ros2_ws/install/fast_livo/share/fast_livo/config/`

| File | Purpose |
|---|---|
| `mid360_hikrobot.yaml` | Main params — extrinsics, LIO/VIO tuning |
| `camera_pinhole_mid360.yaml` | Calibrated camera intrinsics |

**Critical config notes:**
- `ros_driver_bug_fix: false` — ESP32 NTP keeps all clocks aligned, no correction needed
- `img_time_offset: 0.0` — permanent, no per-session recalculation
- `extrinsic_R` values must be floats (`1.0` not `1`) — ROS2 strict typing

---

## Known Issues

| Issue | Fix |
|---|---|
| `libpcl_io libusb_set_option` crash with RViz | `LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0` |
| Camera `OpenDevice fail 0x80000203` | `echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2bdf", MODE="0666"' \| sudo tee /etc/udev/rules.d/99-hikrobot.rules && sudo udevadm control --reload-rules && sudo udevadm trigger` |
| vikit_ros `cannot find -lvikit_common` | Absolute path to libvikit_common.so in CMakeLists — update if workspace moves |

---

## Repository Structure

```
fast_livo2_ros2_jetson/
├── README.md
├── firmware/
│   └── livo2_timesync.ino          ← Arduino Nano ESP32 firmware
├── config/
│   ├── mid360_hikrobot.yaml        ← FAST-LIVO2 main config
│   └── camera_pinhole_mid360.yaml  ← calibrated intrinsics
├── launch/
│   └── mapping_mid360_hikrobot.launch.py
├── mvs_ros2_cam/                   ← HIKrobot ROS2 driver package
│   ├── package.xml
│   ├── CMakeLists.txt
│   ├── src/grab_trigger_ros2.cpp
│   ├── config/cam_config.yaml
│   └── launch/mvs_camera.launch.py
├── patches/
│   ├── vikit_common_CMakeLists.txt
│   └── vikit_ros_CMakeLists.txt
└── scripts/
    └── start_fast_livo2.sh
```
