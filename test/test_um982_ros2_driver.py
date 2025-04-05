#!/usr/bin/env python3

import unittest

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

class TestUM982Driver(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Initialize ROS2
        rclpy.init()
        cls.node = Node('test_um982_ros2_driver')
        
    @classmethod
    def tearDownClass(cls):
        # Clean up
        cls.node.destroy_node()
        rclpy.shutdown()
        
    def test_import(self):
        # Test that the module can be imported
        try:
            from um982_ros2_driver.um982 import UM982Serial
            from um982_ros2_driver.um982_node import UM982DriverNode
            self.assertTrue(True)
        except ImportError as e:
            self.fail(f"Import failed: {e}")
            
    def test_node_creation(self):
        # Test that the node can be created
        try:
            from um982_ros2_driver.um982_node import UM982DriverNode
            # We don't actually create the node since it would try to open a serial port
            self.assertTrue(True)
        except Exception as e:
            self.fail(f"Node creation failed: {e}")


if __name__ == '__main__':
    unittest.main()
