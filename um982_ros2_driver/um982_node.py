import sys
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from .um982 import UM982Serial


class UM982DriverNode(Node):

    def _ros_log_debug(self, log_data):
        self.get_logger().debug(str(log_data))

    def _ros_log_info(self, log_data):
        self.get_logger().info(str(log_data))

    def _ros_log_warn(self, log_data):
        self.get_logger().warn(str(log_data))

    def _ros_log_error(self, log_data):
        self.get_logger().error(str(log_data))

    def __init__(self) -> None:
        super().__init__('um982_ros2_driver_node')
        # Step 1: Get port and baud from parameter server
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 921600)
        self.declare_parameter('frame_id', 'gps')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('publish_rate', 20.0)  # Hz
        
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        # Step 2: Open serial port
        try:
            self.um982serial = UM982Serial(port, baud)
            self._ros_log_info(f'Serial port {port} opened successfully at {baud} baud!')
        except Exception as e:
            self._ros_log_error(f'Failed to open serial port {port}: {str(e)}')
            sys.exit(1)
            
        # Step 3: Start a thread to process serial data
        self.um982serial.start()
        
        # Step 4: ROS related setup
        self.fix_pub = self.create_publisher(NavSatFix, 'gps/fix', 10)
        self.utm_pub = self.create_publisher(Odometry, 'gps/utmpos', 10)
        self.pub_timer = self.create_timer(1.0/publish_rate, self.pub_task)
        
        self._ros_log_info('UM982 driver node initialized')

    def pub_task(self):
        try:
            bestpos_hgt, bestpos_lat, bestpos_lon, bestpos_hgtstd, bestpos_latstd, bestpos_lonstd = self.um982serial.fix
            utm_x, utm_y = self.um982serial.utmpos
            vel_east, vel_north, vel_ver, vel_east_std, vel_north_std, vel_ver_std = self.um982serial.vel
            heading, pitch, roll, quality = self.um982serial.orientation
            this_time = self.get_clock().now().to_msg()

            # PRINT ORIENTATION
            self._ros_log_info(f'Quality (i.e. RTK fix=4): {quality} | Heading: {heading} | Latitude: {bestpos_lat} | Longitude: {bestpos_lon}')

            # Step 1: Publish GPS Fix Data
            fix_msg = NavSatFix()
            fix_msg.header.stamp = this_time
            fix_msg.header.frame_id = self.frame_id
            fix_msg.latitude = bestpos_lat
            fix_msg.longitude = bestpos_lon
            fix_msg.altitude = bestpos_hgt

            # Map quality to NavSatStatus.status
            if quality == 0:
                fix_msg.status.status = NavSatStatus.STATUS_NO_FIX
            elif quality == 4:
                fix_msg.status.status = NavSatStatus.STATUS_GBAS_FIX
            elif quality == 9:
                fix_msg.status.status = NavSatStatus.STATUS_SBAS_FIX
            else:
                fix_msg.status.status = NavSatStatus.STATUS_FIX

            fix_msg.position_covariance[0] = float(bestpos_latstd)**2
            fix_msg.position_covariance[4] = float(bestpos_lonstd)**2
            fix_msg.position_covariance[8] = float(bestpos_hgtstd)**2
            fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            self.fix_pub.publish(fix_msg)

            # Step 2: Publish UTM Position Data
            odom_msg = Odometry()
            odom_msg.header.stamp = this_time
            odom_msg.header.frame_id = 'earth'
            odom_msg.child_frame_id = self.child_frame_id
            odom_msg.pose.pose.position.x = utm_x
            odom_msg.pose.pose.position.y = utm_y
            odom_msg.pose.pose.position.z = bestpos_hgt
            quaternion = quaternion_from_euler(math.radians(roll), math.radians(pitch), math.radians(heading))
            odom_msg.pose.pose.orientation.x = quaternion[0]
            odom_msg.pose.pose.orientation.y = quaternion[1]
            odom_msg.pose.pose.orientation.z = quaternion[2]
            odom_msg.pose.pose.orientation.w = quaternion[3]
            odom_msg.pose.covariance = [0.0] * 36
            odom_msg.pose.covariance[0] = float(bestpos_latstd)**2
            odom_msg.pose.covariance[7] = float(bestpos_lonstd)**2
            odom_msg.pose.covariance[14] = float(bestpos_hgtstd)**2
            odom_msg.pose.covariance[21] = 0.1
            odom_msg.pose.covariance[28] = 0.1
            odom_msg.pose.covariance[35] = 0.1
            odom_msg.twist.twist.linear.x = vel_east
            odom_msg.twist.twist.linear.y = vel_north
            odom_msg.twist.twist.linear.z = vel_ver
            odom_msg.twist.covariance = [0.0] * 36
            odom_msg.twist.covariance[0] = float(vel_east_std)**2
            odom_msg.twist.covariance[7] = float(vel_north_std)**2
            odom_msg.twist.covariance[14] = float(vel_ver_std)**2
            self.utm_pub.publish(odom_msg)
        except Exception as e:
            self._ros_log_error(f'Error in publishing task: {str(e)}')

    def stop(self):
        self._ros_log_info('Shutting down UM982 driver node')
        if hasattr(self, 'um982serial'):
            self.um982serial.stop()
        if hasattr(self, 'pub_timer'):
            self.pub_timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    
    um982_node = UM982DriverNode()
    
    try:
        rclpy.spin(um982_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Unexpected error: {str(e)}')
    finally:
        um982_node.stop()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
