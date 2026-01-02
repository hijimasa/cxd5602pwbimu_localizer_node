# Spresense IMU Localizer ROS 2 Node

English | [日本語](./README_jp.md)

This ROS 2 package provides a node that converts serial output from a Spresense board running [cxd5602pwbimu_localizer_arduino](https://github.com/hijimasa/cxd5602pwbimu_localizer_arduino) into ROS messages. It enables real-time visualization and integration of IMU-based self-localization data with the ROS ecosystem.

## Overview

The node reads serial data from the Spresense board, parses the localization output, and publishes it as standard ROS 2 messages:

- **IMU data** (sensor_msgs/Imu): Angular velocity, linear acceleration, and orientation
- **Pose data** (geometry_msgs/Pose): Position and orientation
- **TF transforms**: Broadcasts the sensor pose as a TF transformation

## Features

- **Serial Communication:** Reads hex-encoded data from Spresense at 115200 baud
- **ROS 2 Integration:** Publishes standard ROS message types for easy integration
- **TF Broadcasting:** Publishes sensor pose as TF transform for visualization in RViz
- **Configurable Parameters:** Serial port, baud rate, and TF frame names are configurable via ROS parameters

## Hardware Requirements

- **Sony Spresense Main Board** with CXD5602 processor
- **Spresense IMU Add-on Board** (CXD5602PWBIMU)
- **USB connection** between Spresense and ROS 2 host

## Software Requirements

- **ROS 2** (Humble or later recommended)
- **Python 3**
- **pyserial** library
- **[cxd5602pwbimu_localizer_arduino](https://github.com/hijimasa/cxd5602pwbimu_localizer_arduino)** firmware running on Spresense

## Installation

### 1. Clone the Repository

```bash
cd ~/ros2_ws/src
git clone https://github.com/hijimasa/cxd5602pwbimu_localizer_node.git
```

### 2. Install Dependencies

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
pip3 install pyserial
```

### 3. Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select cxd5602pwbimu_localizer_node
source install/setup.bash
```

## Usage

### Running the Node

```bash
ros2 run cxd5602pwbimu_localizer_node localizer_node
```

### With Custom Parameters

```bash
ros2 run cxd5602pwbimu_localizer_node localizer_node --ros-args \
  -p serial_port:=/dev/ttyUSB0 \
  -p baud_rate:=115200 \
  -p tf_parent_frame:=world \
  -p tf_child_frame:=sensor
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `serial_port` | `/dev/ttyUSB0` | Serial port connected to Spresense |
| `baud_rate` | `115200` | Serial communication baud rate |
| `tf_parent_frame` | `world` | Parent frame for TF broadcast |
| `tf_child_frame` | `sensor` | Child frame for TF broadcast |

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/imu/data_raw` | `sensor_msgs/Imu` | IMU data (angular velocity, linear acceleration, orientation) |
| `/pose` | `geometry_msgs/Pose` | Sensor pose (position and orientation) |
| `/tf` | `tf2_msgs/TFMessage` | Transform from parent frame to sensor frame |

## Visualization with RViz

1. Launch RViz:
   ```bash
   rviz2
   ```

2. Add displays:
   - Add **TF** display to visualize the sensor frame
   - Add **Pose** display and set topic to `/pose`
   - Add **Imu** display (if rviz_imu_plugin is installed) and set topic to `/imu/data_raw`

3. Set Fixed Frame to `world` (or your configured `tf_parent_frame`)

## Data Format

The node expects serial data in the following hex-encoded, comma-separated format:

```
timestamp, temperature, gx, gy, gz, ax, ay, az, qw, qx, qy, qz, vx, vy, vz, px, py, pz
```

This format is output by the [cxd5602pwbimu_localizer_arduino](https://github.com/hijimasa/cxd5602pwbimu_localizer_arduino) firmware.

## Troubleshooting

### Common Issues

1. **Serial port permission denied:**
   ```bash
   sudo chmod 666 /dev/ttyUSB0
   # Or add user to dialout group (permanent solution):
   sudo usermod -a -G dialout $USER
   ```

2. **Serial port not found:**
   - Check if Spresense is connected: `ls /dev/ttyUSB*`
   - Verify the correct port in parameters

3. **No data received:**
   - Ensure cxd5602pwbimu_localizer_arduino firmware is running on Spresense
   - Check baud rate matches (115200)

## License

Distributed under the MIT License. See `LICENSE` for more information.

## Related Projects

- [cxd5602pwbimu_localizer_arduino](https://github.com/hijimasa/cxd5602pwbimu_localizer_arduino) - Arduino firmware for Spresense IMU self-localization

---

For questions or issues, please open a GitHub issue or contact the repository maintainer.
