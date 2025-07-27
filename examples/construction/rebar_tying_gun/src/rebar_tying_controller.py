#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from std_msgs.msg import Bool, Float32, String
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Point, Twist
from cv_bridge import CvBridge
import threading
import time

class RebarTyingController:
    """Main controller for the rebar tying assistive gun"""
    
    def __init__(self):
        rospy.init_node('rebar_tying_controller', anonymous=True)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # State variables
        self.current_mode = "assisted"  # assisted, semi_auto, manual
        self.is_armed = False
        self.battery_level = 100.0
        self.wire_level = 100.0
        self.tie_count = 0
        self.system_ready = False
        
        # Computer vision variables
        self.current_frame = None
        self.rebar_intersections = []
        self.target_intersection = None
        
        # Publishers
        self.status_pub = rospy.Publisher('/rebar_gun/status', String, queue_size=10)
        self.tie_command_pub = rospy.Publisher('/rebar_gun/tie_command', Bool, queue_size=10)
        self.wire_feed_pub = rospy.Publisher('/rebar_gun/wire_feed', Float32, queue_size=10)
        self.motor_control_pub = rospy.Publisher('/rebar_gun/motor_control', Twist, queue_size=10)
        
        # Subscribers
        self.camera_sub = rospy.Subscriber('/rebar_gun/camera/image_raw', Image, self.camera_callback)
        self.trigger_sub = rospy.Subscriber('/rebar_gun/trigger', Bool, self.trigger_callback)
        self.mode_sub = rospy.Subscriber('/rebar_gun/mode', String, self.mode_callback)
        self.emergency_sub = rospy.Subscriber('/rebar_gun/emergency_stop', Bool, self.emergency_callback)
        
        # Initialize systems
        self.initialize_systems()
        
        rospy.loginfo("Rebar Tying Controller initialized")
        
    def initialize_systems(self):
        """Initialize all subsystems"""
        try:
            # Check hardware connections
            self.check_hardware()
            
            # Initialize computer vision
            self.init_vision_system()
            
            # Initialize motor systems
            self.init_motor_systems()
            
            self.system_ready = True
            self.publish_status("System Ready")
            
        except Exception as e:
            rospy.logerr(f"System initialization failed: {e}")
            self.system_ready = False
            
    def check_hardware(self):
        """Check all hardware components"""
        # Simulate hardware checks
        rospy.loginfo("Checking motor systems...")
        time.sleep(0.5)
        
        rospy.loginfo("Checking wire feeding system...")
        time.sleep(0.5)
        
        rospy.loginfo("Checking sensors...")
        time.sleep(0.5)
        
        rospy.loginfo("Hardware check complete")
        
    def init_vision_system(self):
        """Initialize computer vision system"""
        # Load rebar detection model (simplified)
        self.rebar_detector = RebarDetector()
        rospy.loginfo("Vision system initialized")
        
    def init_motor_systems(self):
        """Initialize motor control systems"""
        # Initialize tying motor
        self.tying_motor_speed = 0.0
        
        # Initialize wire feed motor
        self.wire_feed_speed = 0.0
        
        rospy.loginfo("Motor systems initialized")
        
    def camera_callback(self, msg):
        """Process camera images for rebar detection"""
        try:
            # Convert ROS image to OpenCV
            self.current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Detect rebar intersections
            self.detect_rebar_intersections()
            
        except Exception as e:
            rospy.logerr(f"Camera callback error: {e}")
            
    def detect_rebar_intersections(self):
        """Detect rebar intersections in the current frame"""
        if self.current_frame is None:
            return
            
        # Use the rebar detector
        intersections = self.rebar_detector.detect_intersections(self.current_frame)
        self.rebar_intersections = intersections
        
        # Find the best target intersection
        if intersections:
            self.target_intersection = self.select_target_intersection(intersections)
            
    def select_target_intersection(self, intersections):
        """Select the best target intersection for tying"""
        if not intersections:
            return None
            
        # For now, select the intersection closest to center
        frame_center = (self.current_frame.shape[1]//2, self.current_frame.shape[0]//2)
        
        best_intersection = None
        min_distance = float('inf')
        
        for intersection in intersections:
            distance = np.sqrt((intersection['x'] - frame_center[0])**2 + 
                             (intersection['y'] - frame_center[1])**2)
            if distance < min_distance:
                min_distance = distance
                best_intersection = intersection
                
        return best_intersection
        
    def trigger_callback(self, msg):
        """Handle trigger press"""
        if not self.system_ready:
            rospy.logwarn("System not ready")
            return
            
        if msg.data and self.is_armed:
            self.execute_tie()
            
    def mode_callback(self, msg):
        """Handle mode changes"""
        valid_modes = ["assisted", "semi_auto", "manual"]
        if msg.data in valid_modes:
            self.current_mode = msg.data
            rospy.loginfo(f"Mode changed to: {self.current_mode}")
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
        
        # Stop all motors
        stop_twist = Twist()
        self.motor_control_pub.publish(stop_twist)
        
        # Disable system
        self.is_armed = False
        self.system_ready = False
        
        self.publish_status("EMERGENCY STOP - System Disabled")
        
    def execute_tie(self):
        """Execute the tying sequence"""
        if not self.target_intersection:
            rospy.logwarn("No target intersection found")
            return
            
        rospy.loginfo("Executing tie sequence...")
        
        try:
            # Step 1: Position check
            if not self.verify_position():
                rospy.logwarn("Position verification failed")
                return
                
            # Step 2: Feed wire
            self.feed_wire()
            
            # Step 3: Execute tie
            self.perform_tie()
            
            # Step 4: Cut wire
            self.cut_wire()
            
            # Step 5: Verify tie
            if self.verify_tie():
                self.tie_count += 1
                rospy.loginfo(f"Tie completed successfully. Total ties: {self.tie_count}")
                self.publish_status(f"Tie Complete - Count: {self.tie_count}")
            else:
                rospy.logwarn("Tie verification failed")
                
        except Exception as e:
            rospy.logerr(f"Tie execution failed: {e}")
            
    def verify_position(self):
        """Verify gun is properly positioned at intersection"""
        if not self.target_intersection:
            return False
            
        # Check if intersection is centered
        frame_center = (self.current_frame.shape[1]//2, self.current_frame.shape[0]//2)
        distance = np.sqrt((self.target_intersection['x'] - frame_center[0])**2 + 
                          (self.target_intersection['y'] - frame_center[1])**2)
        
        # Allow 20 pixel tolerance
        return distance < 20
        
    def feed_wire(self):
        """Feed wire for tying"""
        rospy.loginfo("Feeding wire...")
        
        # Send wire feed command
        wire_length = 150.0  # mm
        self.wire_feed_pub.publish(Float32(wire_length))
        
        # Wait for feeding to complete
        time.sleep(1.0)
        
    def perform_tie(self):
        """Perform the actual tying motion"""
        rospy.loginfo("Performing tie...")
        
        # Create tying motion
        tie_twist = Twist()
        tie_twist.angular.z = 2.0  # Rotation speed
        
        # Execute tying rotation
        self.motor_control_pub.publish(tie_twist)
        time.sleep(1.5)  # Time for complete tie
        
        # Stop rotation
        stop_twist = Twist()
        self.motor_control_pub.publish(stop_twist)
        
    def cut_wire(self):
        """Cut the wire after tying"""
        rospy.loginfo("Cutting wire...")
        
        # Send cut command
        self.tie_command_pub.publish(Bool(True))
        time.sleep(0.5)
        
    def verify_tie(self):
        """Verify the tie was completed successfully"""
        # In a real implementation, this would use sensors
        # For now, assume success
        return True
        
    def publish_status(self, message):
        """Publish system status"""
        status_msg = f"Battery: {self.battery_level}% | Wire: {self.wire_level}% | {message}"
        self.status_pub.publish(String(status_msg))
        
    def run(self):
        """Main control loop"""
        rate = rospy.Rate(30)  # 30 Hz
        
        while not rospy.is_shutdown():
            try:
                # Update system status
                self.update_system_status()
                
                # Process current mode
                if self.current_mode == "semi_auto":
                    self.semi_auto_operation()
                elif self.current_mode == "assisted":
                    self.assisted_operation()
                    
                rate.sleep()
                
            except Exception as e:
                rospy.logerr(f"Control loop error: {e}")
                
    def update_system_status(self):
        """Update system status periodically"""
        # Simulate battery drain
        if self.tie_count > 0:
            self.battery_level = max(0, 100 - (self.tie_count * 0.1))
            self.wire_level = max(0, 100 - (self.tie_count * 0.5))
            
    def semi_auto_operation(self):
        """Handle semi-automatic operation mode"""
        if self.target_intersection and self.system_ready:
            # Auto-execute tie when intersection is detected and centered
            if self.verify_position():
                self.execute_tie()
                
    def assisted_operation(self):
        """Handle assisted operation mode"""
        # In assisted mode, wait for manual trigger
        # Provide visual feedback for positioning
        pass

class RebarDetector:
    """Computer vision system for detecting rebar intersections"""
    
    def __init__(self):
        # Initialize detection parameters
        self.min_line_length = 50
        self.max_line_gap = 10
        self.rho = 1
        self.theta = np.pi/180
        self.threshold = 50
        
    def detect_intersections(self, frame):
        """Detect rebar intersections in the frame"""
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Edge detection
        edges = cv2.Canny(blurred, 50, 150)
        
        # Detect lines using Hough transform
        lines = cv2.HoughLinesP(edges, self.rho, self.theta, self.threshold,
                               minLineLength=self.min_line_length,
                               maxLineGap=self.max_line_gap)
        
        # Find intersections
        intersections = []
        if lines is not None:
            intersections = self.find_line_intersections(lines)
            
        return intersections
        
    def find_line_intersections(self, lines):
        """Find intersection points between lines"""
        intersections = []
        
        for i in range(len(lines)):
            for j in range(i+1, len(lines)):
                line1 = lines[i][0]
                line2 = lines[j][0]
                
                # Find intersection point
                intersection = self.line_intersection(line1, line2)
                if intersection:
                    intersections.append({
                        'x': intersection[0],
                        'y': intersection[1],
                        'confidence': 0.8  # Simplified confidence
                    })
                    
        return intersections
        
    def line_intersection(self, line1, line2):
        """Calculate intersection point of two lines"""
        x1, y1, x2, y2 = line1
        x3, y3, x4, y4 = line2
        
        denom = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)
        if abs(denom) < 1e-10:
            return None  # Lines are parallel
            
        t = ((x1-x3)*(y3-y4) - (y1-y3)*(x3-x4)) / denom
        
        # Calculate intersection point
        x = x1 + t*(x2-x1)
        y = y1 + t*(y2-y1)
        
        return (int(x), int(y))

if __name__ == '__main__':
    try:
        controller = RebarTyingController()
        controller.run()
    except rospy.ROSInterruptException:
        pass