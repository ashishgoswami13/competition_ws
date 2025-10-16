#!/usr/bin/env python3
# encoding: utf-8
"""
Line Follower Node for Ainex Humanoid Robot in Gazebo Simulation
This node processes camera images to detect a line and controls the robot to follow it.
Adapted from the real-world visual_patrol_node.py for simulation use.
"""

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, Bool
import math


class LineFollowerController:
    """
    Simplified line following controller for simulation.
    Based on the VisualPatrol class from Reference package but adapted for Gazebo.
    """
    
    def __init__(self):
        # Get parameters from parameter server
        self.x_range = rospy.get_param('~x_range', [0, 0.010])
        self.yaw_range = rospy.get_param('~yaw_range', [-8, 8])
        self.center_x_offset = rospy.get_param('~center_x_offset', 0)
        
        # Gait parameters for walking
        self.period_time = rospy.get_param('~period_time', 400)
        self.body_height = rospy.get_param('~body_height', 0.025)
        self.step_height = rospy.get_param('~step_height', 0.015)
        self.x_max = rospy.get_param('~x_max', 0.010)
        
        # For direct joint control in simulation
        self.step_size = 0.01  # radians per control cycle
        self.current_step_phase = 0.0
        
    def calculate_control(self, line_x, image_width):
        """
        Calculate walking control parameters based on line position.
        
        Args:
            line_x: X coordinate of detected line center
            image_width: Width of the image
            
        Returns:
            tuple: (x_output, yaw_output) - forward step and turning angle
        """
        center_x = image_width / 2 + self.center_x_offset
        
        # Calculate yaw (turning) based on line position
        if abs(line_x - center_x) < 10:
            yaw_output = 0
        elif abs(line_x - center_x) < image_width / 6:
            sign = 1 if (line_x - center_x) > 0 else -1
            yaw_output = sign + self._map_value(
                line_x - center_x, 
                -image_width / 6, 
                image_width / 6,
                self.yaw_range[0] + 1, 
                self.yaw_range[1] - 1
            )
        else:
            yaw_output = self.yaw_range[1] if (line_x - center_x) > 0 else self.yaw_range[0]
        
        # Reduce forward speed when turning
        if abs(yaw_output) > 6:
            x_output = 0.008
        else:
            x_output = self.x_max
            
        return x_output, -yaw_output
    
    def _map_value(self, value, in_min, in_max, out_min, out_max):
        """Map a value from one range to another."""
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


class LineFollowerNode:
    """
    Main node for line following in Gazebo simulation.
    Processes camera images and controls robot joints.
    """
    
    def __init__(self):
        rospy.init_node('line_follower_node', anonymous=False)
        rospy.loginfo("Initializing Line Follower Node for Simulation")
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Line detection parameters
        self.image_process_size = rospy.get_param('~image_process_size', [160, 120])
        self.line_roi = rospy.get_param('~line_roi', [
            [5.0/12.0, 6.0/12.0, 1.0/4.0, 3.0/4.0],
            [6.0/12.0, 7.0/12.0, 1.0/4.0, 3.0/4.0],
            [7.0/12.0, 8.0/12.0, 1.0/4.0, 3.0/4.0]
        ])
        
        # Color threshold for black line detection (HSV)
        self.lower_black = np.array(rospy.get_param('~lower_black', [0, 0, 0]))
        self.upper_black = np.array(rospy.get_param('~upper_black', [180, 255, 50]))
        
        # Control state
        self.enabled = rospy.get_param('~auto_start', True)
        self.controller = LineFollowerController()
        self.line_detected = False
        self.line_x = 0
        self.image_width = self.image_process_size[0]
        
        # Joint publishers for basic walking motion
        self.joint_pubs = {}
        self._setup_joint_publishers()
        
        # Subscribers
        image_topic = rospy.get_param('~image_topic', '/camera/rgb/image_raw')
        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=1)
        
        # Publishers
        self.debug_image_pub = rospy.Publisher('~debug_image', Image, queue_size=1)
        self.line_detected_pub = rospy.Publisher('~line_detected', Bool, queue_size=1)
        
        # Control rate
        self.control_rate = rospy.get_param('~control_rate', 10)
        self.rate = rospy.Rate(self.control_rate)
        
        # Initial pose - prepare to walk
        self.init_walking_pose()
        
        rospy.loginfo("Line Follower Node initialized successfully")
    
    def _setup_joint_publishers(self):
        """Setup publishers for all joint controllers."""
        joints = [
            'l_hip_yaw', 'l_hip_roll', 'l_hip_pitch', 'l_knee', 'l_ank_pitch', 'l_ank_roll',
            'r_hip_yaw', 'r_hip_roll', 'r_hip_pitch', 'r_knee', 'r_ank_pitch', 'r_ank_roll',
            'l_sho_pitch', 'r_sho_pitch'
        ]
        
        for joint in joints:
            topic = '/{}_controller/command'.format(joint)
            self.joint_pubs[joint] = rospy.Publisher(topic, Float64, queue_size=1)
    
    def init_walking_pose(self):
        """Initialize robot to walking ready pose."""
        rospy.loginfo("Setting initial walking pose...")
        rospy.sleep(0.5)  # Wait for publishers to connect
        
        # Set all leg joints to standing position (neutral)
        init_positions = {
            'l_hip_yaw': 0.0,
            'l_hip_roll': 0.0,
            'l_hip_pitch': 0.0,
            'l_knee': 0.0,
            'l_ank_pitch': 0.0,
            'l_ank_roll': 0.0,
            'r_hip_yaw': 0.0,
            'r_hip_roll': 0.0,
            'r_hip_pitch': 0.0,
            'r_knee': 0.0,
            'r_ank_pitch': 0.0,
            'r_ank_roll': 0.0,
            'l_sho_pitch': 0.0,
            'r_sho_pitch': 0.0,
        }
        
        for joint, position in init_positions.items():
            if joint in self.joint_pubs:
                self.joint_pubs[joint].publish(Float64(position))
        
        rospy.sleep(1.0)
        rospy.loginfo("Initial pose set")
    
    def image_callback(self, msg):
        """Process incoming camera images to detect the line."""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CV Bridge Error: {}".format(e))
            return
        
        # Resize image for processing
        resized = cv2.resize(cv_image, tuple(self.image_process_size))
        
        # Detect line
        line_detected, line_x, debug_image = self.detect_line(resized)
        
        # Update state
        self.line_detected = line_detected
        if line_detected:
            self.line_x = line_x
            self.image_width = self.image_process_size[0]
        
        # Publish debug image
        if self.debug_image_pub.get_num_connections() > 0:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, "bgr8")
                self.debug_image_pub.publish(debug_msg)
            except CvBridgeError as e:
                rospy.logerr("CV Bridge Error in debug: {}".format(e))
        
        # Publish detection status
        self.line_detected_pub.publish(Bool(line_detected))
    
    def detect_line(self, image):
        """
        Detect black line in the image using HSV color space.
        
        Args:
            image: Input image (BGR format)
            
        Returns:
            tuple: (line_detected, line_x, debug_image)
        """
        # Convert to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Create mask for black color
        mask = cv2.inRange(hsv, self.lower_black, self.upper_black)
        
        # Process each ROI
        debug_image = image.copy()
        line_detected = False
        line_x = 0
        
        for i, roi in enumerate(self.line_roi):
            y_min = int(roi[0] * self.image_process_size[1])
            y_max = int(roi[1] * self.image_process_size[1])
            x_min = int(roi[2] * self.image_process_size[0])
            x_max = int(roi[3] * self.image_process_size[0])
            
            # Draw ROI on debug image
            cv2.rectangle(debug_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 1)
            
            # Extract ROI from mask
            roi_mask = mask[y_min:y_max, x_min:x_max]
            
            # Find contours in ROI
            contours, _ = cv2.findContours(roi_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                # Find largest contour
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)
                
                if area > 50:  # Minimum area threshold
                    # Calculate center of contour
                    M = cv2.moments(largest_contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"]) + x_min
                        cy = int(M["m01"] / M["m00"]) + y_min
                        
                        # Draw contour and center point
                        cv2.drawContours(debug_image, [largest_contour + np.array([x_min, y_min])], -1, (0, 0, 255), 2)
                        cv2.circle(debug_image, (cx, cy), 5, (255, 0, 0), -1)
                        
                        # Use center ROI (index 1) for line position
                        if i == 1:
                            line_detected = True
                            line_x = cx
                        
                        break  # Exit if line found in any ROI
        
        # Draw center line
        center_x = int(self.image_process_size[0] / 2)
        cv2.line(debug_image, (center_x, 0), (center_x, self.image_process_size[1]), (255, 255, 0), 1)
        
        # Add text
        status_text = "Line Detected: {}".format("YES" if line_detected else "NO")
        cv2.putText(debug_image, status_text, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        if line_detected:
            pos_text = "Line X: {}".format(line_x)
            cv2.putText(debug_image, pos_text, (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        return line_detected, line_x, debug_image
    
    def control_walking(self):
        """Execute walking control based on line detection."""
        if not self.enabled:
            return
        
        if self.line_detected:
            # Calculate control parameters
            x_output, yaw_output = self.controller.calculate_control(self.line_x, self.image_width)
            
            # Simple walking motion simulation
            # In a real implementation, this would use the full walking module
            # For now, we'll just adjust hip yaw joints based on turning direction
            
            # Scale yaw for joint movement
            yaw_radians = math.radians(yaw_output * 0.5)  # Scale down for safety
            
            # Apply symmetric yaw to hips for turning
            self.joint_pubs['l_hip_yaw'].publish(Float64(yaw_radians))
            self.joint_pubs['r_hip_yaw'].publish(Float64(-yaw_radians))
            
            rospy.loginfo_throttle(1.0, "Following line - X: {}, Yaw: {:.2f}°".format(
                self.line_x, yaw_output))
        else:
            # No line detected - stop or search
            rospy.logwarn_throttle(2.0, "No line detected!")
    
    def run(self):
        """Main control loop."""
        rospy.loginfo("Line follower node running...")
        
        while not rospy.is_shutdown():
            self.control_walking()
            self.rate.sleep()


def main():
    try:
        node = LineFollowerNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Line follower node terminated.")


if __name__ == '__main__':
    main()
