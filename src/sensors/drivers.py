#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan, Image, Imu
import numpy as np

class SensorDriver:
    """Base class for sensor drivers"""
    def __init__(self, name, topic):
        self.name = name
        self.topic = topic
        self.data = None
        self.timestamp = None

    def get_data(self):
        """Get latest sensor data"""
        return self.data, self.timestamp

class LidarDriver(SensorDriver):
    """Driver for LiDAR sensors"""
    def __init__(self, name, topic="/scan"):
        super().__init__(name, topic)
        self.angle_min = -np.pi
        self.angle_max = np.pi
        self.angle_increment = np.pi/180  # 1 degree
        self.scan_time = 0.1
        self.range_min = 0.1
        self.range_max = 30.0
        
    def process_scan(self, scan_msg):
        """Process incoming LiDAR scan"""
        self.data = scan_msg.ranges
        self.timestamp = scan_msg.header.stamp

class CameraDriver(SensorDriver):
    """Driver for camera sensors"""
    def __init__(self, name, topic="/camera/image_raw"):
        super().__init__(name, topic)
        self.width = 640
        self.height = 480
        self.encoding = "rgb8"
        
    def process_image(self, image_msg):
        """Process incoming camera image"""
        self.data = image_msg.data
        self.timestamp = image_msg.header.stamp

class IMUDriver(SensorDriver):
    """Driver for IMU sensors"""
    def __init__(self, name, topic="/imu/data"):
        super().__init__(name, topic)
        self.angular_velocity = [0, 0, 0]
        self.linear_acceleration = [0, 0, 0]
        self.orientation = [0, 0, 0, 1]  # quaternion
        
    def process_imu(self, imu_msg):
        """Process incoming IMU data"""
        self.angular_velocity = [
            imu_msg.angular_velocity.x,
            imu_msg.angular_velocity.y,
            imu_msg.angular_velocity.z
        ]
        self.linear_acceleration = [
            imu_msg.linear_acceleration.x,
            imu_msg.linear_acceleration.y,
            imu_msg.linear_acceleration.z
        ]
        self.orientation = [
            imu_msg.orientation.x,
            imu_msg.orientation.y,
            imu_msg.orientation.z,
            imu_msg.orientation.w
        ]
        self.timestamp = imu_msg.header.stamp 