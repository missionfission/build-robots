#!/usr/bin/env python3

import rospy
import tf2_ros
import numpy as np
from std_msgs.msg import Bool, Float32, String, Int32
from sensor_msgs.msg import JointState, Image, PointCloud2
from geometry_msgs.msg import Pose, PoseStamped, Twist, WrenchStamped
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from cv_bridge import CvBridge
import cv2
import threading
import time

class PlasteringArmController:
    """Main controller for the teleoperated plastering robot arm"""
    
    def __init__(self):
        rospy.init_node('plastering_arm_controller', anonymous=True)
        
        # Initialize MoveIt
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.arm_group = MoveGroupCommander("arm")
        self.gripper_group = MoveGroupCommander("gripper")
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # State variables
        self.current_mode = "teleop"  # teleop, assisted, semi_auto, teaching
        self.system_ready = False
        self.is_plastering = False
        self.emergency_stop_active = False
        
        # Teleoperation variables
        self.teleop_enabled = False
        self.force_feedback_enabled = True
        self.current_tool = "spray"  # spray, trowel
        
        # Plastering parameters
        self.plaster_thickness = 15.0  # mm
        self.spray_pressure = 4.0  # bar
        self.material_flow_rate = 2.0  # L/min
        self.surface_speed = 0.1  # m/s
        
        # Surface analysis
        self.current_surface = None
        self.surface_normal = None
        self.target_points = []
        
        # Publishers
        self.joint_command_pub = rospy.Publisher('/arm/joint_commands', JointState, queue_size=10)
        self.tool_control_pub = rospy.Publisher('/plastering/tool_control', String, queue_size=10)
        self.spray_control_pub = rospy.Publisher('/plastering/spray_control', Float32, queue_size=10)
        self.status_pub = rospy.Publisher('/plastering/status', String, queue_size=10)
        self.force_feedback_pub = rospy.Publisher('/teleop/force_feedback', WrenchStamped, queue_size=10)
        
        # Subscribers
        self.teleop_cmd_sub = rospy.Subscriber('/teleop/arm_command', Pose, self.teleop_callback)
        self.mode_sub = rospy.Subscriber('/plastering/mode', String, self.mode_callback)
        self.emergency_sub = rospy.Subscriber('/plastering/emergency_stop', Bool, self.emergency_callback)
        self.surface_sub = rospy.Subscriber('/sensors/surface_scan', PointCloud2, self.surface_callback)
        self.force_sensor_sub = rospy.Subscriber('/sensors/force_torque', WrenchStamped, self.force_callback)
        self.camera_sub = rospy.Subscriber('/cameras/surface_camera', Image, self.camera_callback)
        
        # Initialize systems
        self.initialize_systems()
        
        rospy.loginfo("Plastering Arm Controller initialized")
        
    def initialize_systems(self):
        """Initialize all subsystems"""
        try:
            # Initialize arm to home position
            self.move_to_home_position()
            
            # Initialize tool systems
            self.init_tool_systems()
            
            # Initialize surface analysis
            self.surface_analyzer = SurfaceAnalyzer()
            
            # Initialize teleoperation interface
            self.teleop_interface = TeleopInterface()
            
            self.system_ready = True
            self.publish_status("System Ready - Plastering Arm Online")
            
        except Exception as e:
            rospy.logerr(f"System initialization failed: {e}")
            self.system_ready = False
            
    def move_to_home_position(self):
        """Move arm to home/safe position"""
        rospy.loginfo("Moving to home position...")
        
        # Define home joint positions
        home_joints = [0.0, -1.57, 1.57, 0.0, 1.57, 0.0]  # Example joint angles
        
        self.arm_group.set_joint_value_target(home_joints)
        plan = self.arm_group.plan()
        
        if plan[0]:  # If planning succeeded
            self.arm_group.execute(plan[1], wait=True)
            rospy.loginfo("Moved to home position")
        else:
            rospy.logwarn("Failed to plan to home position")
            
    def init_tool_systems(self):
        """Initialize plastering tool systems"""
        # Initialize spray system
        self.spray_system = SpraySystem()
        
        # Initialize trowel system
        self.trowel_system = TrowelSystem()
        
        # Initialize material mixing system
        self.mixer_system = MaterialMixer()
        
        rospy.loginfo("Tool systems initialized")
        
    def teleop_callback(self, msg):
        """Handle teleoperation commands"""
        if not self.teleop_enabled or not self.system_ready:
            return
            
        try:
            # Convert pose command to joint space
            target_pose = msg
            
            # Set target pose for arm
            self.arm_group.set_pose_target(target_pose)
            
            # Plan and execute movement
            plan = self.arm_group.plan()
            if plan[0]:
                self.arm_group.execute(plan[1], wait=False)
                
        except Exception as e:
            rospy.logerr(f"Teleoperation error: {e}")
            
    def mode_callback(self, msg):
        """Handle mode changes"""
        valid_modes = ["teleop", "assisted", "semi_auto", "teaching"]
        if msg.data in valid_modes:
            self.current_mode = msg.data
            rospy.loginfo(f"Mode changed to: {self.current_mode}")
            
            # Configure mode-specific settings
            if msg.data == "teleop":
                self.teleop_enabled = True
            elif msg.data == "teaching":
                self.start_teaching_mode()
                
            self.publish_status(f"Mode: {self.current_mode}")
        else:
            rospy.logwarn(f"Invalid mode: {msg.data}")
            
    def emergency_callback(self, msg):
        """Handle emergency stop"""
        if msg.data:
            self.emergency_stop()
            
    def emergency_stop(self):
        """Execute emergency stop procedure"""
        rospy.logwarn("EMERGENCY STOP ACTIVATED")
        
        # Stop all arm movement
        self.arm_group.stop()
        
        # Stop all tool operations
        self.stop_all_tools()
        
        # Disable teleoperation
        self.teleop_enabled = False
        self.emergency_stop_active = True
        
        self.publish_status("EMERGENCY STOP - All Systems Disabled")
        
    def surface_callback(self, msg):
        """Process surface scan data"""
        try:
            # Analyze surface for plastering
            self.current_surface = self.surface_analyzer.process_pointcloud(msg)
            
            if self.current_surface:
                # Calculate surface normal
                self.surface_normal = self.surface_analyzer.calculate_normal(self.current_surface)
                
                # Generate target points for plastering
                self.target_points = self.surface_analyzer.generate_target_points(
                    self.current_surface, self.plaster_thickness)
                    
        except Exception as e:
            rospy.logerr(f"Surface processing error: {e}")
            
    def force_callback(self, msg):
        """Process force/torque sensor data"""
        if self.force_feedback_enabled:
            # Send force feedback to teleop interface
            self.force_feedback_pub.publish(msg)
            
            # Check for excessive forces
            force_magnitude = np.sqrt(msg.wrench.force.x**2 + 
                                    msg.wrench.force.y**2 + 
                                    msg.wrench.force.z**2)
            
            if force_magnitude > 50.0:  # N
                rospy.logwarn("Excessive force detected - reducing speed")
                self.reduce_movement_speed()
                
    def camera_callback(self, msg):
        """Process camera images for visual feedback"""
        try:
            # Convert to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Analyze surface quality
            quality_score = self.analyze_surface_quality(cv_image)
            
            if quality_score < 0.7:  # Poor quality threshold
                rospy.logwarn("Poor surface quality detected")
                
        except Exception as e:
            rospy.logerr(f"Camera processing error: {e}")
            
    def start_plastering_sequence(self):
        """Start automated plastering sequence"""
        if not self.system_ready or not self.target_points:
            rospy.logwarn("Cannot start plastering - system not ready or no targets")
            return
            
        rospy.loginfo("Starting plastering sequence...")
        self.is_plastering = True
        
        try:
            # Select appropriate tool
            if self.current_tool == "spray":
                self.execute_spray_plastering()
            elif self.current_tool == "trowel":
                self.execute_trowel_plastering()
                
        except Exception as e:
            rospy.logerr(f"Plastering sequence failed: {e}")
            self.is_plastering = False
            
    def execute_spray_plastering(self):
        """Execute spray plastering sequence"""
        rospy.loginfo("Executing spray plastering...")
        
        # Start spray system
        self.spray_system.start_spray(self.spray_pressure, self.material_flow_rate)
        
        # Move through target points
        for i, target_point in enumerate(self.target_points):
            if not self.is_plastering:  # Check for stop command
                break
                
            # Move to target position
            self.move_to_target_point(target_point)
            
            # Apply plaster
            self.apply_plaster_at_point(target_point)
            
            # Update progress
            progress = (i + 1) / len(self.target_points) * 100
            self.publish_status(f"Plastering Progress: {progress:.1f}%")
            
        # Stop spray system
        self.spray_system.stop_spray()
        
        rospy.loginfo("Spray plastering completed")
        self.is_plastering = False
        
    def execute_trowel_plastering(self):
        """Execute trowel plastering sequence"""
        rospy.loginfo("Executing trowel plastering...")
        
        # Switch to trowel tool
        self.switch_tool("trowel")
        
        # Move through target points with trowel motion
        for target_point in self.target_points:
            if not self.is_plastering:
                break
                
            # Move to target
            self.move_to_target_point(target_point)
            
            # Execute trowel motion
            self.execute_trowel_motion(target_point)
            
        rospy.loginfo("Trowel plastering completed")
        self.is_plastering = False
        
    def move_to_target_point(self, target_point):
        """Move arm to target point with proper orientation"""
        # Create pose with surface normal orientation
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position.x = target_point['x']
        target_pose.pose.position.y = target_point['y']
        target_pose.pose.position.z = target_point['z']
        
        # Set orientation perpendicular to surface
        if self.surface_normal:
            # Calculate orientation from surface normal
            target_pose.pose.orientation = self.calculate_orientation_from_normal(self.surface_normal)
        
        # Move to target
        self.arm_group.set_pose_target(target_pose.pose)
        plan = self.arm_group.plan()
        
        if plan[0]:
            self.arm_group.execute(plan[1], wait=True)
        else:
            rospy.logwarn("Failed to plan to target point")
            
    def apply_plaster_at_point(self, target_point):
        """Apply plaster at specific point"""
        # Adjust spray parameters based on surface
        if target_point.get('roughness', 0) > 0.5:
            # Increase flow for rough surfaces
            flow_rate = self.material_flow_rate * 1.2
        else:
            flow_rate = self.material_flow_rate
            
        # Apply plaster for calculated duration
        application_time = self.calculate_application_time(target_point)
        
        self.spray_system.set_flow_rate(flow_rate)
        time.sleep(application_time)
        
    def execute_trowel_motion(self, target_point):
        """Execute trowel smoothing motion"""
        # Calculate trowel path
        trowel_path = self.calculate_trowel_path(target_point)
        
        # Execute smoothing motion
        for waypoint in trowel_path:
            waypoint_pose = PoseStamped()
            waypoint_pose.pose.position.x = waypoint['x']
            waypoint_pose.pose.position.y = waypoint['y']
            waypoint_pose.pose.position.z = waypoint['z']
            
            self.arm_group.set_pose_target(waypoint_pose.pose)
            plan = self.arm_group.plan()
            
            if plan[0]:
                self.arm_group.execute(plan[1], wait=True)
                
    def switch_tool(self, tool_name):
        """Switch between spray and trowel tools"""
        rospy.loginfo(f"Switching to {tool_name} tool...")
        
        # Send tool change command
        self.tool_control_pub.publish(String(f"switch_{tool_name}"))
        
        # Wait for tool change to complete
        time.sleep(3.0)
        
        self.current_tool = tool_name
        rospy.loginfo(f"Tool switched to {tool_name}")
        
    def start_teaching_mode(self):
        """Start teaching mode for recording operator movements"""
        rospy.loginfo("Starting teaching mode...")
        
        # Enable gravity compensation
        self.enable_gravity_compensation()
        
        # Start recording joint positions
        self.recorded_trajectory = []
        self.recording = True
        
        # Start recording thread
        self.recording_thread = threading.Thread(target=self.record_trajectory)
        self.recording_thread.start()
        
    def record_trajectory(self):
        """Record arm trajectory during teaching"""
        rate = rospy.Rate(10)  # 10 Hz recording
        
        while self.recording and not rospy.is_shutdown():
            try:
                # Get current joint states
                current_joints = self.arm_group.get_current_joint_values()
                current_pose = self.arm_group.get_current_pose().pose
                
                # Record waypoint
                waypoint = {
                    'joints': current_joints,
                    'pose': current_pose,
                    'timestamp': rospy.Time.now().to_sec()
                }
                
                self.recorded_trajectory.append(waypoint)
                
                rate.sleep()
                
            except Exception as e:
                rospy.logerr(f"Recording error: {e}")
                
    def stop_all_tools(self):
        """Stop all tool operations"""
        self.spray_system.stop_spray()
        self.trowel_system.stop()
        self.mixer_system.stop()
        
    def reduce_movement_speed(self):
        """Reduce arm movement speed for safety"""
        self.arm_group.set_max_velocity_scaling_factor(0.3)
        self.arm_group.set_max_acceleration_scaling_factor(0.3)
        
    def analyze_surface_quality(self, image):
        """Analyze surface quality from camera image"""
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Calculate texture metrics
        texture_score = cv2.Laplacian(gray, cv2.CV_64F).var()
        
        # Normalize to 0-1 range
        quality_score = min(texture_score / 1000.0, 1.0)
        
        return quality_score
        
    def calculate_orientation_from_normal(self, normal):
        """Calculate orientation quaternion from surface normal"""
        # Simplified orientation calculation
        # In practice, this would use proper rotation matrices
        from tf.transformations import quaternion_from_euler
        
        # Calculate euler angles from normal vector
        roll = 0.0
        pitch = np.arctan2(normal[0], normal[2])
        yaw = np.arctan2(normal[1], normal[2])
        
        return quaternion_from_euler(roll, pitch, yaw)
        
    def calculate_application_time(self, target_point):
        """Calculate plaster application time based on surface properties"""
        base_time = 2.0  # seconds
        
        # Adjust based on surface roughness
        if 'roughness' in target_point:
            roughness_factor = 1.0 + target_point['roughness']
            return base_time * roughness_factor
            
        return base_time
        
    def calculate_trowel_path(self, target_point):
        """Calculate trowel smoothing path"""
        # Generate circular smoothing pattern
        center_x, center_y, center_z = target_point['x'], target_point['y'], target_point['z']
        radius = 0.05  # 5cm radius
        
        path = []
        for angle in np.linspace(0, 2*np.pi, 20):
            x = center_x + radius * np.cos(angle)
            y = center_y + radius * np.sin(angle)
            z = center_z
            
            path.append({'x': x, 'y': y, 'z': z})
            
        return path
        
    def enable_gravity_compensation(self):
        """Enable gravity compensation for teaching mode"""
        # This would interface with the robot's gravity compensation system
        rospy.loginfo("Gravity compensation enabled")
        
    def publish_status(self, message):
        """Publish system status"""
        status_msg = f"Plastering Arm | Tool: {self.current_tool} | {message}"
        self.status_pub.publish(String(status_msg))
        
    def run(self):
        """Main control loop"""
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            try:
                # Monitor system health
                self.monitor_system_health()
                
                # Handle current mode operations
                if self.current_mode == "semi_auto" and not self.is_plastering:
                    if self.target_points:
                        self.start_plastering_sequence()
                        
                rate.sleep()
                
            except Exception as e:
                rospy.logerr(f"Control loop error: {e}")
                
    def monitor_system_health(self):
        """Monitor system health and safety"""
        # Check for emergency conditions
        if self.emergency_stop_active:
            return
            
        # Monitor joint limits
        current_joints = self.arm_group.get_current_joint_values()
        joint_limits = self.arm_group.get_active_joints()
        
        # Check for joint limit violations
        # (Implementation would check actual joint limits)

class SurfaceAnalyzer:
    """Analyzes surfaces for plastering operations"""
    
    def process_pointcloud(self, pointcloud_msg):
        """Process point cloud data to extract surface information"""
        # Simplified surface processing
        # In practice, this would use PCL or similar library
        return {
            'points': [],  # Surface points
            'normal': [0, 0, 1],  # Surface normal
            'roughness': 0.3  # Surface roughness metric
        }
        
    def calculate_normal(self, surface):
        """Calculate surface normal vector"""
        return surface.get('normal', [0, 0, 1])
        
    def generate_target_points(self, surface, thickness):
        """Generate target points for plastering"""
        # Generate grid of target points
        targets = []
        for x in np.arange(-0.5, 0.5, 0.1):
            for y in np.arange(-0.5, 0.5, 0.1):
                targets.append({
                    'x': x,
                    'y': y,
                    'z': 0.5,  # Example height
                    'roughness': 0.3
                })
        return targets

class SpraySystem:
    """Controls the spray plastering system"""
    
    def __init__(self):
        self.is_spraying = False
        self.current_pressure = 0.0
        self.current_flow_rate = 0.0
        
    def start_spray(self, pressure, flow_rate):
        """Start spray operation"""
        self.current_pressure = pressure
        self.current_flow_rate = flow_rate
        self.is_spraying = True
        rospy.loginfo(f"Spray started - Pressure: {pressure} bar, Flow: {flow_rate} L/min")
        
    def stop_spray(self):
        """Stop spray operation"""
        self.is_spraying = False
        self.current_pressure = 0.0
        self.current_flow_rate = 0.0
        rospy.loginfo("Spray stopped")
        
    def set_flow_rate(self, flow_rate):
        """Adjust flow rate"""
        self.current_flow_rate = flow_rate

class TrowelSystem:
    """Controls the trowel system"""
    
    def __init__(self):
        self.is_active = False
        
    def stop(self):
        """Stop trowel operations"""
        self.is_active = False

class MaterialMixer:
    """Controls the material mixing system"""
    
    def __init__(self):
        self.is_mixing = False
        
    def stop(self):
        """Stop mixing operations"""
        self.is_mixing = False

class TeleopInterface:
    """Handles teleoperation interface"""
    
    def __init__(self):
        rospy.loginfo("Teleoperation interface initialized")

if __name__ == '__main__':
    try:
        controller = PlasteringArmController()
        controller.run()
    except rospy.ROSInterruptException:
        pass