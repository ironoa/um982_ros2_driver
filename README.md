# UM982 Driver for ROS2

This package provides a ROS2 driver for the [UNICORECOMM UM982](https://en.unicore.com/products/dual-antenna-gnss-um982/)/UM980 GPS device. It reads data from the GPS device and publishes it as ROS2 messages.

## Features

- Reads GPS data from UNICORECOMM UM982/UM980 devices
- Supports NMEA and extended NMEA instruction sets: `PVTSLN`, `GNHPR`, `BESTNAV`
- Publishes GPS data as standard ROS2 messages:
  - `sensor_msgs/NavSatFix` for GPS position
  - `nav_msgs/Odometry` for UTM position and velocity

## Installation

### Dependencies

- ROS2 Jazzy
- Python packages:
  - pyproj
  - pyserial

### Building from Source

```bash
# Clone the repository into your ROS2 workspace
cd ~/ros2_ws/src
git clone https://github.com/ironoa/um982_ros2_driver.git

# Install dependencies
sudo apt-get update && \
    apt-get install -y \
    ros-jazzy-tf-transformations \
    python3-pyproj \
    python3-serial

# Build the package
cd ~/ros2_ws
colcon build --packages-select um982_ros2_driver

# Source the workspace
source ~/ros2_ws/install/setup.bash
```

## Usage

### Configuration

Before using the driver, you need to configure the UM982 device to output the required data. Connect to the device and send the following commands:

```
config com3 115200
PVTSLNA com3 0.05
GPHPR com3 0.05
BESTNAVA com3 0.05
```

This configures the device to output the required data on COM2 at 115200 baud rate, with a frequency of 0.05 seconds (20 Hz).

### Running the Node

You can run the node using the provided launch file:

```bash
ros2 launch um982_ros2_driver um982_ros2_driver.launch.py port:=/dev/ttyUSB0 baud:=115200
```

### Parameters

The node supports the following parameters:

- `port` (string, default: `/dev/ttyUSB0`): Serial port for the UM982 GPS device
- `baud` (int, default: `921600`): Baud rate for the serial connection
- `frame_id` (string, default: `gps`): Frame ID for the GPS messages
- `child_frame_id` (string, default: `base_link`): Child frame ID for the odometry message
- `publish_rate` (double, default: `20.0`): Rate at which to publish GPS data (Hz)

### Published Topics

- `/gps/fix` (sensor_msgs/NavSatFix): GPS position data
- `/gps/utmpos` (nav_msgs/Odometry): UTM position and velocity data

## License

This package is licensed under the GPL License.

## Acknowledgements

This driver is based on the [UM982Driver](https://github.com/sunshineharry/UM982Driver) by sunshineharry.
