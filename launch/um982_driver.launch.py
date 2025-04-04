import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('um982_ros2_driver')
    
    # Path to the config file
    config_file = os.path.join(pkg_dir, 'config', 'um982_params.yaml')
    
    # Declare launch arguments for overriding parameters
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for the UM982 device'
    )

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='921600',
        description='Baud rate for the serial connection'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='um982',
        description='Frame ID for the UM982 messages'
    )

    # Create node
    um982_driver_node = Node(
        package='um982_ros2_driver',
        executable='um982_driver_node',
        name='um982_driver',
        parameters=[
            config_file,  # Load parameters from config file
            {  # Override with launch arguments if provided
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'frame_id': LaunchConfiguration('frame_id'),
            }
        ],
        output='screen'
    )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        frame_id_arg,
        um982_driver_node
    ])
