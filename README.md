# UM982 ROS 2 Driver

A ROS 2 driver for the UM982 GNSS module. This driver provides a ROS 2 interface to the UM982 GNSS module, publishing position, velocity, and orientation data.

## Features

- Threaded implementation for continuous serial communication
- Comprehensive message parsing for:
  - PVTSLN messages (position data)
  - GNHPR messages (heading, pitch, roll)
  - BESTNAV messages (navigation data)
  - KSXT messages (direct velocity measurements)
- Coordinate transformations (WGS84 to UTM)
- Thread-safe data access with locks
- ROS 2 lifecycle node implementation
- Configurable parameters via YAML file

## Published Topics

- `/um982_driver/fix` (NavSatFix): GPS position data
- `/um982_driver/imu` (Imu): Orientation data
- `/um982_driver/twist` (TwistWithCovarianceStamped): Velocity data
- `/um982_driver/odom` (Odometry): Combined position and velocity data

## Parameters

All parameters can be configured in the `config/um982_params.yaml` file:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| serial_port | string | /dev/ttyUSB0 | Serial port for the UM982 device |
| baud_rate | int | 921600 | Baud rate for the serial connection |
| frame_id | string | um982 | Frame ID for the UM982 messages |
| publish_tf | bool | true | Whether to publish TF transforms |
| publish_rate | float | 10.0 | Rate at which to publish messages (Hz) |
| prefer_ksxt | bool | true | Whether to prefer KSXT messages over BESTNAV for velocity data |

## Installation

### Dependencies

- ROS 2 (tested on Humble)
- Python 3.8+
- pyserial
- numpy
- pyproj

### Building from Source

```bash
# Create a ROS 2 workspace (if you don't already have one)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone the repository
git clone https://github.com/your-username/um982_ros2_driver.git

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build the package
colcon build --packages-select um982_ros2_driver

# Source the workspace
source install/setup.bash
```

## Usage

### Basic Usage

```bash
# Launch the driver with default parameters
ros2 launch um982_ros2_driver um982_driver.launch.py
```

### Overriding Parameters

```bash
# Override the serial port and baud rate
ros2 launch um982_ros2_driver um982_driver.launch.py serial_port:=/dev/ttyACM0 baud_rate:=115200
```

### Viewing Data

```bash
# View position data
ros2 topic echo /um982_driver/fix

# View velocity data
ros2 topic echo /um982_driver/twist

# View orientation data
ros2 topic echo /um982_driver/imu

# View combined data
ros2 topic echo /um982_driver/odom
```

## License

Apache License 2.0

## Acknowledgements

This driver is based on the work from the lostDeers and sunshine projects.
