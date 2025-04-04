"""
ROS 2 node for the UM982 GNSS device.

This module contains the ROS 2 node implementation for the UM982 GNSS device.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.parameter import Parameter
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn

from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from geometry_msgs.msg import TwistWithCovarianceStamped, QuaternionStamped, Vector3
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from .driver.um982_driver import UM982Driver


class UM982Node(LifecycleNode):
    """
    ROS 2 node for the UM982 GNSS device.
    
    This class implements a ROS 2 lifecycle node for the UM982 GNSS device.
    It publishes position, velocity, and orientation data from the device.
    """
    
    def __init__(self):
        """
        Initialize the UM982Node.
        """
        super().__init__('um982_driver')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 921600)
        self.declare_parameter('frame_id', 'um982')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_rate', 10.0)
        
        # Initialize variables
        self.driver = None
        self.publishers = {}
        self.tf_broadcaster = None
        self.timer = None
        
        # QoS profile for sensor data
        self.sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

    def on_configure(self, state):
        """
        Configure the node.
        
        This callback is called when the lifecycle node is being configured.
        
        Args:
            state: Current lifecycle state
            
        Returns:
            TransitionCallbackReturn: SUCCESS if configuration successful
        """
        self.get_logger().info('Configuring UM982 driver node')
        
        # Get parameters
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        
        # Log parameters
        self.get_logger().info(f'Serial port: {serial_port}')
        self.get_logger().info(f'Baud rate: {baud_rate}')
        self.get_logger().info(f'Frame ID: {self.get_parameter("frame_id").value}')
        self.get_logger().info(f'Publish TF: {self.get_parameter("publish_tf").value}')
        self.get_logger().info(f'Publish rate: {self.get_parameter("publish_rate").value}')
        
        # Create driver
        self.driver = UM982Driver(serial_port, baud_rate)
        
        # Create publishers
        self.publishers['fix'] = self.create_lifecycle_publisher(
            NavSatFix,
            'fix',
            self.sensor_qos
        )
        
        self.publishers['imu'] = self.create_lifecycle_publisher(
            Imu,
            'imu',
            self.sensor_qos
        )
        
        self.publishers['twist'] = self.create_lifecycle_publisher(
            TwistWithCovarianceStamped,
            'twist',
            self.sensor_qos
        )
        
        self.publishers['odom'] = self.create_lifecycle_publisher(
            Odometry,
            'odom',
            self.sensor_qos
        )
        
        # Create TF broadcaster if needed
        if self.get_parameter('publish_tf').value:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        """
        Activate the node.
        
        This callback is called when the lifecycle node is being activated.
        
        Args:
            state: Current lifecycle state
            
        Returns:
            TransitionCallbackReturn: SUCCESS if activation successful
        """
        self.get_logger().info('Activating UM982 driver node')
        
        # Start driver
        if not self.driver.start():
            self.get_logger().error('Failed to start UM982 driver')
            return TransitionCallbackReturn.FAILURE
        
        # Create timer for publishing data
        publish_rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(
            1.0 / publish_rate,
            self.publish_data
        )
        
        # Activate publishers
        for publisher in self.publishers.values():
            publisher.on_activate()
        
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        """
        Deactivate the node.
        
        This callback is called when the lifecycle node is being deactivated.
        
        Args:
            state: Current lifecycle state
            
        Returns:
            TransitionCallbackReturn: SUCCESS if deactivation successful
        """
        self.get_logger().info('Deactivating UM982 driver node')
        
        # Deactivate publishers
        for publisher in self.publishers.values():
            publisher.on_deactivate()
        
        # Stop timer
        if self.timer:
            self.timer.cancel()
            self.timer = None
        
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        """
        Clean up the node.
        
        This callback is called when the lifecycle node is being cleaned up.
        
        Args:
            state: Current lifecycle state
            
        Returns:
            TransitionCallbackReturn: SUCCESS if cleanup successful
        """
        self.get_logger().info('Cleaning up UM982 driver node')
        
        # Stop driver
        if self.driver:
            self.driver.stop()
            self.driver = None
        
        # Reset publishers
        self.publishers = {}
        
        # Reset TF broadcaster
        self.tf_broadcaster = None
        
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        """
        Shut down the node.
        
        This callback is called when the lifecycle node is being shut down.
        
        Args:
            state: Current lifecycle state
            
        Returns:
            TransitionCallbackReturn: SUCCESS if shutdown successful
        """
        self.get_logger().info('Shutting down UM982 driver node')
        
        # Stop driver
        if self.driver:
            self.driver.stop()
            self.driver = None
        
        return TransitionCallbackReturn.SUCCESS

    def publish_data(self):
        """
        Publish data from the UM982 device.
        
        This function is called periodically to publish position, velocity,
        and orientation data from the UM982 device.
        """
        if not self.driver or not self.driver.is_ready():
            return
        
        # Get current time
        now = self.get_clock().now().to_msg()
        
        # Get frame ID
        frame_id = self.get_parameter('frame_id').value
        
        # Publish NavSatFix
        self.publish_navsatfix(now, frame_id)
        
        # Publish IMU
        self.publish_imu(now, frame_id)
        
        # Publish TwistWithCovarianceStamped
        self.publish_twist(now, frame_id)
        
        # Publish Odometry
        self.publish_odometry(now, frame_id)
        
        # Publish TF if needed
        if self.tf_broadcaster:
            self.publish_tf(now, frame_id)

    def publish_navsatfix(self, timestamp, frame_id):
        """
        Publish NavSatFix message.
        
        Args:
            timestamp: ROS 2 timestamp
            frame_id: Frame ID for the message
        """
        lat, lon, alt, lat_std, lon_std, alt_std = self.driver.get_position()
        
        msg = NavSatFix()
        msg.header.stamp = timestamp
        msg.header.frame_id = frame_id
        
        # Set status
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        
        # Set position
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt
        
        # Set covariance
        # Diagonal matrix with variances
        msg.position_covariance[0] = lat_std ** 2  # Variance for latitude
        msg.position_covariance[4] = lon_std ** 2  # Variance for longitude
        msg.position_covariance[8] = alt_std ** 2  # Variance for altitude
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        
        # Publish message
        self.publishers['fix'].publish(msg)

    def publish_imu(self, timestamp, frame_id):
        """
        Publish IMU message.
        
        Args:
            timestamp: ROS 2 timestamp
            frame_id: Frame ID for the message
        """
        heading, pitch, roll = self.driver.get_orientation()
        
        msg = Imu()
        msg.header.stamp = timestamp
        msg.header.frame_id = frame_id
        
        # Convert Euler angles to quaternion
        # Note: The UM982 provides heading (yaw), pitch, and roll in degrees
        # We need to convert to radians for the quaternion conversion
        heading_rad = math.radians(heading)
        pitch_rad = math.radians(pitch)
        roll_rad = math.radians(roll)
        
        # Convert to quaternion (assuming ZYX rotation order)
        cy = math.cos(heading_rad * 0.5)
        sy = math.sin(heading_rad * 0.5)
        cp = math.cos(pitch_rad * 0.5)
        sp = math.sin(pitch_rad * 0.5)
        cr = math.cos(roll_rad * 0.5)
        sr = math.sin(roll_rad * 0.5)
        
        msg.orientation.w = cy * cp * cr + sy * sp * sr
        msg.orientation.x = cy * cp * sr - sy * sp * cr
        msg.orientation.y = sy * cp * sr + cy * sp * cr
        msg.orientation.z = sy * cp * cr - cy * sp * sr
        
        # We don't have angular velocity or linear acceleration data
        # Set covariance to unknown
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0
        
        # Publish message
        self.publishers['imu'].publish(msg)

    def publish_twist(self, timestamp, frame_id):
        """
        Publish TwistWithCovarianceStamped message.
        
        Args:
            timestamp: ROS 2 timestamp
            frame_id: Frame ID for the message
        """
        vel_e, vel_n, vel_u, vel_e_std, vel_n_std, vel_u_std = self.driver.get_velocity()
        
        msg = TwistWithCovarianceStamped()
        msg.header.stamp = timestamp
        msg.header.frame_id = frame_id
        
        # Set linear velocity
        # Note: ROS uses x forward, y left, z up
        # UM982 provides east, north, up
        # So we need to convert: east -> y, north -> x, up -> z
        msg.twist.twist.linear.x = vel_n  # North -> x
        msg.twist.twist.linear.y = vel_e  # East -> y
        msg.twist.twist.linear.z = vel_u  # Up -> z
        
        # Set covariance (6x6 matrix: [x, y, z, rx, ry, rz])
        # We only have linear velocity, so we'll set the top-left 3x3 block
        # Diagonal elements are variances
        msg.twist.covariance[0] = vel_n_std ** 2  # Variance for x (north)
        msg.twist.covariance[7] = vel_e_std ** 2  # Variance for y (east)
        msg.twist.covariance[14] = vel_u_std ** 2  # Variance for z (up)
        
        # Publish message
        self.publishers['twist'].publish(msg)

    def publish_odometry(self, timestamp, frame_id):
        """
        Publish Odometry message.
        
        Args:
            timestamp: ROS 2 timestamp
            frame_id: Frame ID for the message
        """
        # Get position and velocity data
        utm_x, utm_y, utm_z = self.driver.get_utm_position()
        vel_e, vel_n, vel_u, vel_e_std, vel_n_std, vel_u_std = self.driver.get_velocity()
        heading, pitch, roll = self.driver.get_orientation()
        
        msg = Odometry()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'map'  # Parent frame
        msg.child_frame_id = frame_id  # Child frame
        
        # Set position (UTM coordinates)
        msg.pose.pose.position.x = utm_x
        msg.pose.pose.position.y = utm_y
        msg.pose.pose.position.z = utm_z
        
        # Set orientation (quaternion)
        heading_rad = math.radians(heading)
        pitch_rad = math.radians(pitch)
        roll_rad = math.radians(roll)
        
        cy = math.cos(heading_rad * 0.5)
        sy = math.sin(heading_rad * 0.5)
        cp = math.cos(pitch_rad * 0.5)
        sp = math.sin(pitch_rad * 0.5)
        cr = math.cos(roll_rad * 0.5)
        sr = math.sin(roll_rad * 0.5)
        
        msg.pose.pose.orientation.w = cy * cp * cr + sy * sp * sr
        msg.pose.pose.orientation.x = cy * cp * sr - sy * sp * cr
        msg.pose.pose.orientation.y = sy * cp * sr + cy * sp * cr
        msg.pose.pose.orientation.z = sy * cp * cr - cy * sp * sr
        
        # Set linear velocity
        # Note: ROS uses x forward, y left, z up
        # UM982 provides east, north, up
        # So we need to convert: east -> y, north -> x, up -> z
        msg.twist.twist.linear.x = vel_n  # North -> x
        msg.twist.twist.linear.y = vel_e  # East -> y
        msg.twist.twist.linear.z = vel_u  # Up -> z
        
        # Set covariance (6x6 matrix: [x, y, z, rx, ry, rz])
        # We only have linear velocity, so we'll set the top-left 3x3 block
        # Diagonal elements are variances
        msg.twist.covariance[0] = vel_n_std ** 2  # Variance for x (north)
        msg.twist.covariance[7] = vel_e_std ** 2  # Variance for y (east)
        msg.twist.covariance[14] = vel_u_std ** 2  # Variance for z (up)
        
        # Publish message
        self.publishers['odom'].publish(msg)

    def publish_tf(self, timestamp, frame_id):
        """
        Publish TF transform.
        
        Args:
            timestamp: ROS 2 timestamp
            frame_id: Frame ID for the message
        """
        # Get position and orientation data
        utm_x, utm_y, utm_z = self.driver.get_utm_position()
        heading, pitch, roll = self.driver.get_orientation()
        
        # Create transform message
        transform = TransformStamped()
        transform.header.stamp = timestamp
        transform.header.frame_id = 'map'  # Parent frame
        transform.child_frame_id = frame_id  # Child frame
        
        # Set translation
        transform.transform.translation.x = utm_x
        transform.transform.translation.y = utm_y
        transform.transform.translation.z = utm_z
        
        # Set rotation (quaternion)
        heading_rad = math.radians(heading)
        pitch_rad = math.radians(pitch)
        roll_rad = math.radians(roll)
        
        cy = math.cos(heading_rad * 0.5)
        sy = math.sin(heading_rad * 0.5)
        cp = math.cos(pitch_rad * 0.5)
        sp = math.sin(pitch_rad * 0.5)
        cr = math.cos(roll_rad * 0.5)
        sr = math.sin(roll_rad * 0.5)
        
        transform.transform.rotation.w = cy * cp * cr + sy * sp * sr
        transform.transform.rotation.x = cy * cp * sr - sy * sp * cr
        transform.transform.rotation.y = sy * cp * sr + cy * sp * cr
        transform.transform.rotation.z = sy * cp * cr - cy * sp * sr
        
        # Publish transform
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    """
    Main function.
    
    Args:
        args: Command line arguments
    """
    rclpy.init(args=args)
    
    node = UM982Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
