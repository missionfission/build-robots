#!/usr/bin/env python3

import rospy
import tf2_ros
import numpy as np
from geometry_msgs.msg import Transform, Vector3, Quaternion
from visualization_msgs.msg import Marker, MarkerArray

class WorldModel:
    """Manages the robot's world model and object tracking"""
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.marker_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
        self.objects = {}  # Dictionary to store world objects
        
    def add_object(self, obj_id, position, orientation, obj_type="cube", color=(1,0,0,1)):
        """Add an object to the world model"""
        self.objects[obj_id] = {
            'position': position,
            'orientation': orientation,
            'type': obj_type,
            'color': color
        }
        self._publish_markers()
        
    def update_object(self, obj_id, position=None, orientation=None):
        """Update object pose in the world model"""
        if obj_id in self.objects:
            if position is not None:
                self.objects[obj_id]['position'] = position
            if orientation is not None:
                self.objects[obj_id]['orientation'] = orientation
            self._publish_markers()
            
    def remove_object(self, obj_id):
        """Remove an object from the world model"""
        if obj_id in self.objects:
            del self.objects[obj_id]
            self._publish_markers()
            
    def get_object_pose(self, obj_id):
        """Get the pose of an object"""
        if obj_id in self.objects:
            return (
                self.objects[obj_id]['position'],
                self.objects[obj_id]['orientation']
            )
        return None
        
    def _publish_markers(self):
        """Publish visualization markers for all objects"""
        marker_array = MarkerArray()
        
        for obj_id, obj in self.objects.items():
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "world_objects"
            marker.id = hash(obj_id) % 65535
            
            # Set marker type
            if obj['type'] == "cube":
                marker.type = Marker.CUBE
            elif obj['type'] == "sphere":
                marker.type = Marker.SPHERE
            elif obj['type'] == "cylinder":
                marker.type = Marker.CYLINDER
                
            # Set pose
            marker.pose.position = Vector3(*obj['position'])
            marker.pose.orientation = Quaternion(*obj['orientation'])
            
            # Set scale and color
            marker.scale = Vector3(0.1, 0.1, 0.1)  # Default size
            marker.color.r = obj['color'][0]
            marker.color.g = obj['color'][1]
            marker.color.b = obj['color'][2]
            marker.color.a = obj['color'][3]
            
            marker_array.markers.append(marker)
            
        self.marker_pub.publish(marker_array)

class ObstacleDetector:
    """Detects and tracks obstacles in the environment"""
    def __init__(self, world_model):
        self.world_model = world_model
        self.min_obstacle_size = 0.1
        
    def process_lidar_scan(self, ranges, angle_min, angle_increment):
        """Process LiDAR scan to detect obstacles"""
        obstacles = []
        for i, r in enumerate(ranges):
            if r < self.min_obstacle_size:
                continue
            angle = angle_min + i * angle_increment
            x = r * np.cos(angle)
            y = r * np.sin(angle)
            obstacles.append((x, y))
        return obstacles

class TerrainMapper:
    """Maps and analyzes terrain features"""
    def __init__(self, resolution=0.05):
        self.resolution = resolution
        self.height_map = {}
        
    def update_point(self, x, y, z):
        """Update height map with a new point"""
        grid_x = int(x / self.resolution)
        grid_y = int(y / self.resolution)
        self.height_map[(grid_x, grid_y)] = z
        
    def get_height(self, x, y):
        """Get interpolated height at given position"""
        grid_x = int(x / self.resolution)
        grid_y = int(y / self.resolution)
        return self.height_map.get((grid_x, grid_y), 0.0) 