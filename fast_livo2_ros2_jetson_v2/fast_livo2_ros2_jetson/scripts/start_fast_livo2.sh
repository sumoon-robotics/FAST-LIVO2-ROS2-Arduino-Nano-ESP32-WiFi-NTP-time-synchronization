#!/bin/bash
# start_fast_livo2.sh
# One-shot startup for FAST-LIVO2 on Jetson AGX Orin
# Arduino Nano ESP32 handles timesync — no clock offset calculation needed.
#
# Usage:
#   ~/start_fast_livo2.sh          # no RViz
#   ~/start_fast_livo2.sh --rviz   # with RViz2

set -e

USE_RVIZ="False"
[[ "$1" == "--rviz" ]] && USE_RVIZ="True"

WS=/mnt/ssd/fast_livo2_ros2_ws
LIVOX_WS=~/ws_livox

source /opt/ros/humble/setup.bash
source $LIVOX_WS/install/setup.bash
source $WS/install/setup.bash

echo "================================================"
echo " FAST-LIVO2 ROS2 — Jetson AGX Orin"
echo " Arduino Nano ESP32 timesync active"
echo "================================================"

# ── Verify ESP32 is connected ─────────────────────────────────────────
if ! timeout 2 cat /dev/ttyUSB0 2>/dev/null | grep -q "GPRMC"; then
  echo "WARNING: No GPRMC on /dev/ttyUSB0 — check ESP32 connection"
else
  echo "[OK] ESP32 GPRMC active on /dev/ttyUSB0"
fi

# ── Step 1: LiDAR ─────────────────────────────────────────────────────
echo "[1/3] Starting Livox Mid-360..."
ros2 launch livox_ros_driver2 msg_MID360_launch.py &
LIDAR_PID=$!
sleep 8
echo "[OK] LiDAR running"

# ── Step 2: Camera ────────────────────────────────────────────────────
echo "[2/3] Starting HIKrobot camera..."
ros2 launch mvs_ros2_cam mvs_camera.launch.py &
CAM_PID=$!
sleep 3
echo "[OK] Camera running"

# ── Step 3: FAST-LIVO2 ───────────────────────────────────────────────
echo "[3/3] Launching FAST-LIVO2 (use_rviz=$USE_RVIZ)..."
echo ""
echo "  Hold sensor FLAT and STILL for 5 seconds after start."
echo "  Then move slowly — LIO initializes, then VIO kicks in."
echo ""

# LD_PRELOAD fixes libpcl_io libusb conflict on Jetson aarch64
LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libusb-1.0.so.0 \
  ros2 launch fast_livo mapping_mid360_hikrobot.launch.py use_rviz:=$USE_RVIZ

# Cleanup on Ctrl+C
kill $LIDAR_PID $CAM_PID 2>/dev/null || true
