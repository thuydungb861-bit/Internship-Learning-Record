# hwt906p_driver

ROS 2 Humble C++ driver for the WitMotion HWT906P IMU.

## Topics

- `/imu/data` -> `sensor_msgs/msg/Imu`
- `/imu/mag` -> `sensor_msgs/msg/MagneticField`

## Build

```bash
cd /home/wzz/hwt906p_ros2/hwt906p_ros2_ws
sudo apt install ros-humble-rclcpp ros-humble-sensor-msgs
source /opt/ros/humble/setup.bash
colcon build --packages-select hwt906p_driver
```

## Run

```bash
cd /home/wzz/hwt906p_ros2/hwt906p_ros2_ws
source /opt/ros/humble/setup.bash
source /home/wzz/hwt906p_ros2/hwt906p_ros2_ws/install/setup.bash
ros2 launch hwt906p_driver hwt906p.launch.py
```

Or:

```bash
cd /home/wzz/hwt906p_ros2/hwt906p_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run hwt906p_driver hwt906p_node --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=230400
```

Or let the node auto-scan common baud rates:

```bash
cd /home/wzz/hwt906p_ros2/hwt906p_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run hwt906p_driver hwt906p_node --ros-args -p port:=/dev/ttyUSB0 -p baudrate_candidates:="[230400, 115200, 9600, 57600, 38400, 19200]"
```

## Notes

- The parser expects the default WitMotion 11-byte frames with header `0x55`.
- Linear acceleration is converted to `m/s^2`.
- Angular velocity is converted to `rad/s`.
- Quaternion frame `0x59` is preferred; if not present, the node falls back to Euler-to-quaternion conversion.
- Magnetic-field scaling differs across firmware variants, so `magnetic_field_lsb_to_tesla` is exposed as a parameter.
- Use `/imu/data` for downstream processing or external tools.
- If the serial device disappears or is busy during startup, the node keeps retrying instead of exiting.
- You can tune reconnect behavior with `reconnect_interval_sec`, for example `-p reconnect_interval_sec:=0.5`.
- If no valid `0x55` frames are parsed for `frame_timeout_sec`, the node automatically tries the next baud rate in `baudrate_candidates`.
- The node prints serial byte and valid-sample counters every `status_log_interval_sec`, which helps distinguish "no bytes", "wrong baudrate", and "frames are flowing".
