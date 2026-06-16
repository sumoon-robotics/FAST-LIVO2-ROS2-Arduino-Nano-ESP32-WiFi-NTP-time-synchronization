# Arduino Nano ESP32 Wiring Reference

## Pin assignments

| ESP32 Pin | Signal | Destination | Notes |
|---|---|---|---|
| D1 (TX1) | GPRMC 9600 baud | Mid-360 Sync Cable — Gray/White | Serial UART1 |
| D4 (TX2) | GPRMC mirror 9600 baud | USB-TTL Adapter RX → Jetson `/dev/ttyUSB0` | For monitoring/verification |
| D5 | PPS 1Hz square wave | Mid-360 Sync Cable — Purple/White | 100ms HIGH, 900ms LOW |
| A0 | Camera trigger 10Hz PWM | HIKrobot I/O Cable — Line 2 (+) | 10ms pulse width |
| GND | Ground | Mid-360 Black, HIKrobot Line 2 (−), USB-TTL GND | Common ground |
| USB-C | Power + programming | Jetson USB port | Also USB CDC serial at 115200 |

## Mid-360 sync cable colors

| Wire color | Signal | Connect to |
|---|---|---|
| Gray / White | GPRMC input (9600 baud) | ESP32 D1 |
| Purple / White | PPS input (1Hz) | ESP32 D5 |
| Black | GND | ESP32 GND |

## HIKrobot MV-CA016-10UC I/O cable

| Signal | Connect to |
|---|---|
| Line 2 trigger (+) | ESP32 A0 |
| Line 2 trigger (−) | ESP32 GND |
| USB cable (separate) | Jetson USB port — image data |

## USB-TTL adapter

| TTL pin | Connect to |
|---|---|
| RX | ESP32 D4 |
| GND | ESP32 GND |
| USB | Jetson USB port → `/dev/ttyUSB0` |

## Verify wiring on Jetson

```bash
# GPRMC arriving from ESP32 via USB-TTL
timeout 5 cat /dev/ttyUSB0
# Expected: $GPRMC,HHMMSS.000,A,0.0,N,0.0,E,0.0,0.0,DDMMYY,0.0,,A*XX

# Mid-360 clock synced (should match system time)
source /opt/ros/humble/setup.bash && source ~/ws_livox/install/setup.bash
ros2 topic echo /livox/imu --once | grep "sec:"
date +%s
# Both numbers should match within 1-2 seconds
```
