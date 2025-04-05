from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port for the UM982 GPS device'
    )
    
    baud_arg = DeclareLaunchArgument(
        'baud',
        default_value='115200',
        description='Baud rate for the serial connection'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='gps',
        description='Frame ID for the GPS messages'
    )
    
    child_frame_id_arg = DeclareLaunchArgument(
        'child_frame_id',
        default_value='base_link',
        description='Child frame ID for the odometry message'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='20.0',
        description='Rate at which to publish GPS data (Hz)'
    )
    
    # Create the node
    um982_node = Node(
        package='um982_ros2_driver',
        executable='um982_node',
        name='um982_ros2_driver_node',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baud': LaunchConfiguration('baud'),
            'frame_id': LaunchConfiguration('frame_id'),
            'child_frame_id': LaunchConfiguration('child_frame_id'),
            'publish_rate': LaunchConfiguration('publish_rate'),
        }],
        output='screen'
    )
    
    # Return the launch description
    return LaunchDescription([
        port_arg,
        baud_arg,
        frame_id_arg,
        child_frame_id_arg,
        publish_rate_arg,
        um982_node
    ])
