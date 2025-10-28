#!/usr/bin/env python3
# encoding: utf-8

"""
Line Follower Node V3 - With Head Stabilization and Search Behavior
Features:
1. Flexible neck with damping for stability (reduces jerkiness)
2. Head search behavior when line is lost
3. Smooth line following with adaptive speed
"""

import rospy
import cv2
import numpy as np
import math
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from ainex_kinematics.gait_manager import GaitManager
from ainex_kinematics.motion_manager import MotionManager

def imgmsg_to_cv2(img_msg):
    """Convert ROS Image message to OpenCV image"""
    if img_msg.encoding == "rgb8":
        img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, 3)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    elif img_msg.encoding == "bgr8":
        img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, 3)
    elif img_msg.encoding == "mono8":
        img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width)
    else:
        rospy.logerr(f"Unsupported encoding: {img_msg.encoding}")
        return None
    return img

def cv2_to_imgmsg(cv_image, encoding="bgr8"):
    """Convert OpenCV image to ROS Image message"""
    img_msg = Image()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = encoding
    img_msg.step = cv_image.shape[1] * cv_image.shape[2] if len(cv_image.shape) > 2 else cv_image.shape[1]
    img_msg.data = cv_image.tobytes()
    return img_msg


class LineFollowerV3:
    """
    Advanced Line Follower with:
    - Head stabilization (flexible neck)
    - Head search when line is lost
    - Smooth walking control
    """
    
    # Head servo IDs (from ainex_controller.py)
    HEAD_PAN_ID = 23   # Left-right rotation
    HEAD_TILT_ID = 24  # Up-down rotation
    
    def __init__(self):
        rospy.init_node('line_follower_node', anonymous=False)
        rospy.loginfo("🚀 Line Follower V3 Starting (with head stabilization)...")
        
        # ROS Parameters
        self.image_topic = rospy.get_param('~image_topic', '/camera/image_raw')
        self.control_rate = rospy.get_param('~control_rate', 10)
        self.image_process_size = rospy.get_param('~image_process_size', [160, 120])
        self.line_roi = rospy.get_param('~line_roi', [[0.5, 0.7, 0.2, 0.8]])
        self.lower_black = np.array(rospy.get_param('~lower_black', [0, 0, 0]))
        self.upper_black = np.array(rospy.get_param('~upper_black', [180, 255, 50]))
        self.min_contour_area = rospy.get_param('~min_contour_area', 50)
        
        # Walking control parameters
        self.center_tolerance = rospy.get_param('~center_tolerance', 10)
        self.max_turn_angle = rospy.get_param('~max_turn_angle', 8.0)
        self.min_turn_angle = rospy.get_param('~min_turn_angle', 4.0)
        self.max_forward_speed = rospy.get_param('~forward_speed', 0.010)
        self.min_forward_speed = rospy.get_param('~min_forward_speed', 0.006)
        
        # Head control parameters
        self.head_pan_center = 500    # Center position (pulse width)
        self.head_tilt_center = 380   # Slightly downward to see line
        self.head_pan_range = [125, 875]    # Full range
        self.head_tilt_range = [300, 500]   # Limited range for stability
        self.head_search_speed = 300  # Duration in ms for head movements
        
        # Search pattern when line is lost (pan positions)
        self.search_positions = [
            500,  # Center
            650,  # Left
            350,  # Right
            750,  # Far left
            250,  # Far right
        ]
        self.search_index = 0
        
        # State tracking
        self.current_image = None
        self.line_detected = False
        self.line_center_x = 0
        self.last_line_x = None
        self.line_lost_count = 0
        self.max_line_lost_frames = 30  # Search for 3 seconds before stopping
        self.is_initialized = False
        self.first_step_done = False
        self.is_searching = False
        self.search_start_time = None
        
        # Initialize Managers
        rospy.loginfo("⚙️  Initializing GaitManager and MotionManager...")
        self.gait_manager = GaitManager()
        self.motion_manager = MotionManager()
        
        # Configure walking parameters - SLOW and STABLE
        self.walking_param = self.gait_manager.get_gait_param()
        self.walking_param['body_height'] = 0.025
        self.walking_param['step_height'] = 0.010
        self.walking_param['hip_pitch_offset'] = 15
        self.walking_param['z_swap_amplitude'] = 0.005
        
        # DSP parameters [period_ms, dsp_ratio, y_swap]
        self.straight_dsp = [600, 0.2, 0.02]
        self.turn_dsp = [600, 0.2, 0.02]
        self.arm_swing = 0  # No arm swing for stability
        
        # Publishers
        self.debug_image_pub = rospy.Publisher('/line_follower_node/debug_image', Image, queue_size=1)
        self.line_detected_pub = rospy.Publisher('/line_follower_node/line_detected', Bool, queue_size=1)
        
        # Subscriber
        self.image_sub = rospy.Subscriber(
            self.image_topic, Image, self.image_callback,
            queue_size=1, buff_size=2**24
        )
        
        rospy.loginfo("✅ Line Follower V3 Ready!")
        
    def image_callback(self, msg):
        """Store latest image"""
        self.current_image = msg
        
    def set_head_position(self, pan, tilt, duration=300):
        """
        Set head position using MotionManager
        Args:
            pan: Pan position (pulse width 125-875)
            tilt: Tilt position (pulse width 300-500)
            duration: Movement duration in ms
        """
        try:
            self.motion_manager.set_servos_position(duration, [
                [self.HEAD_PAN_ID, pan],
                [self.HEAD_TILT_ID, tilt]
            ])
        except Exception as e:
            rospy.logwarn(f"Head position set warning: {e}")
            
    def initialize_robot(self):
        """Gradual robot initialization with head stabilization"""
        rospy.loginfo("⏳ Initializing robot - waiting for simulation...")
        rospy.sleep(2.0)
        
        rospy.loginfo("🤖 Setting initial standing pose...")
        try:
            self.gait_manager.update_pose(self.walking_param)
            rospy.sleep(1.0)
        except Exception as e:
            rospy.logwarn(f"Pose update warning: {e}")
        
        # Set head to center position looking down at line
        rospy.loginfo("👀 Positioning head to look at line...")
        self.set_head_position(self.head_pan_center, self.head_tilt_center, duration=500)
        rospy.sleep(0.8)
        
        rospy.loginfo("✅ Robot initialized and ready!")
        self.is_initialized = True
        
    def detect_line(self, image):
        """
        Detect black line in image
        Returns: (line_detected, line_center_x, debug_image)
        """
        height, width = self.image_process_size[1], self.image_process_size[0]
        resized = cv2.resize(image, (width, height))
        debug_image = resized.copy()
        
        hsv = cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_black, self.upper_black)
        
        line_detected = False
        line_center_x = 0
        largest_contour_area = 0
        
        for roi in self.line_roi:
            y_min = int(roi[0] * height)
            y_max = int(roi[1] * height)
            x_min = int(roi[2] * width)
            x_max = int(roi[3] * width)
            
            cv2.rectangle(debug_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 1)
            roi_mask = mask[y_min:y_max, x_min:x_max]
            
            contour_result = cv2.findContours(roi_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contour_result) == 3:
                _, contours, _ = contour_result
            else:
                contours, _ = contour_result
            
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                contour_area = cv2.contourArea(largest_contour)
                
                if contour_area > self.min_contour_area and contour_area > largest_contour_area:
                    largest_contour_area = contour_area
                    M = cv2.moments(largest_contour)
                    
                    if M["m00"] > 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        line_center_x = cx + x_min
                        line_detected = True
                        
                        adjusted_contour = largest_contour + np.array([x_min, y_min])
                        cv2.drawContours(debug_image, [adjusted_contour], -1, (0, 0, 255), 2)
                        cv2.circle(debug_image, (line_center_x, cy + y_min), 5, (255, 0, 0), -1)
        
        cv2.line(debug_image, (width // 2, 0), (width // 2, height), (0, 255, 255), 1)
        
        # Add search status to debug image
        if self.is_searching:
            status_text = "SEARCHING..."
            status_color = (0, 165, 255)  # Orange
        elif line_detected:
            status_text = "Line: YES"
            status_color = (0, 255, 0)
        else:
            status_text = "Line: NO"
            status_color = (0, 0, 255)
            
        cv2.putText(debug_image, status_text, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, status_color, 1)
        
        if line_detected:
            cv2.putText(debug_image, f"X: {line_center_x}", (5, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        return line_detected, line_center_x, debug_image
    
    def search_for_line(self):
        """
        Execute head search pattern to find lost line
        Returns: True if should continue searching, False if should stop
        """
        if not self.is_searching:
            # Just started searching
            self.is_searching = True
            self.search_start_time = rospy.Time.now()
            self.search_index = 0
            rospy.loginfo("🔍 Starting head search for line...")
        
        # Check if we've been searching too long
        search_duration = (rospy.Time.now() - self.search_start_time).to_sec()
        if search_duration > 5.0:  # 5 second timeout
            rospy.logwarn("❌ Search timeout - line not found")
            self.is_searching = False
            return False
        
        # Move head to next search position
        pan_position = self.search_positions[self.search_index]
        self.set_head_position(pan_position, self.head_tilt_center, duration=self.head_search_speed)
        
        rospy.loginfo_throttle(0.5, f"🔍 Searching position {self.search_index + 1}/{len(self.search_positions)}: pan={pan_position}")
        
        # Move to next position
        self.search_index = (self.search_index + 1) % len(self.search_positions)
        
        # Brief pause while searching
        rospy.sleep(0.3)
        
        return True
    
    def on_line_found(self):
        """Called when line is found after searching"""
        if self.is_searching:
            rospy.loginfo("✅ Line found! Returning head to center...")
            self.is_searching = False
            self.search_index = 0
            # Return head to center position
            self.set_head_position(self.head_pan_center, self.head_tilt_center, duration=400)
            rospy.sleep(0.5)
    
    def calculate_turn_angle(self, line_x, image_width):
        """Calculate turn angle based on line position"""
        center_x = image_width / 2.0
        error = line_x - center_x
        
        if abs(error) < self.center_tolerance:
            return 0
        elif abs(error) < image_width / 6:
            turn_ratio = abs(error) / (image_width / 6)
            turn_angle = self.min_turn_angle + turn_ratio * (self.max_turn_angle - self.min_turn_angle)
            return math.copysign(turn_angle, error)
        else:
            return math.copysign(self.max_turn_angle, error)
    
    def calculate_forward_speed(self, turn_angle):
        """Calculate forward speed based on turn angle"""
        abs_turn = abs(turn_angle)
        
        if abs_turn < self.min_turn_angle:
            return self.max_forward_speed
        elif abs_turn < self.max_turn_angle:
            turn_ratio = (abs_turn - self.min_turn_angle) / (self.max_turn_angle - self.min_turn_angle)
            speed = self.max_forward_speed - turn_ratio * (self.max_forward_speed - self.min_forward_speed)
            return speed
        else:
            return self.min_forward_speed
    
    def execute_walk_command(self, forward_speed, turn_angle, is_turning):
        """Execute walking command using GaitManager"""
        dsp = self.turn_dsp if is_turning else self.straight_dsp
        
        self.gait_manager.set_step(
            dsp,
            forward_speed,
            0,
            int(-turn_angle),
            self.walking_param,
            arm_swap=self.arm_swing,
            step_num=0
        )
    
    def control_walking(self):
        """Main walking control with search behavior"""
        if not self.line_detected:
            # Line not detected
            self.line_lost_count += 1
            
            if self.line_lost_count < self.max_line_lost_frames and self.last_line_x is not None:
                # Search for line with head
                should_continue = self.search_for_line()
                
                if should_continue:
                    # Keep moving forward very slowly while searching
                    rospy.logwarn_throttle(1.0, f"⚠️  Searching for line ({self.line_lost_count}/{self.max_line_lost_frames})...")
                    self.execute_walk_command(
                        forward_speed=self.min_forward_speed * 0.4,  # Very slow
                        turn_angle=0,
                        is_turning=False
                    )
                else:
                    # Search timeout - stop
                    if self.first_step_done:
                        self.gait_manager.stop()
            else:
                # Lost line for too long - stop
                if self.first_step_done:
                    rospy.logwarn_throttle(2.0, "❌ Line lost - STOPPED")
                    self.gait_manager.stop()
                else:
                    rospy.logwarn_throttle(2.0, "❌ No line detected - waiting...")
            return
        
        # Line detected!
        self.on_line_found()  # Handle search completion
        self.line_lost_count = 0
        self.last_line_x = self.line_center_x
        
        # First step - very gentle
        if not self.first_step_done:
            rospy.loginfo("🚶 Taking first step gently...")
            self.execute_walk_command(
                forward_speed=0.003,
                turn_angle=0,
                is_turning=False
            )
            self.first_step_done = True
            rospy.sleep(1.0)
            return
        
        # Normal line following
        image_width = self.image_process_size[0]
        turn_angle = self.calculate_turn_angle(self.line_center_x, image_width)
        forward_speed = self.calculate_forward_speed(turn_angle)
        is_turning = abs(turn_angle) >= self.min_turn_angle
        
        self.execute_walk_command(forward_speed, turn_angle, is_turning)
        
        direction = "⬆️  STRAIGHT" if abs(turn_angle) < 1 else f"↩️  TURN {turn_angle:.1f}°"
        rospy.loginfo_throttle(1.0,
            f"✅ {direction} | Speed: {forward_speed*1000:.1f}mm | Line X: {self.line_center_x}")
    
    def run(self):
        """Main execution loop"""
        rate = rospy.Rate(self.control_rate)
        
        # Initialize robot
        self.initialize_robot()
        
        # Main control loop
        while not rospy.is_shutdown():
            if self.current_image is not None:
                cv_image = imgmsg_to_cv2(self.current_image)
                
                if cv_image is not None:
                    # Detect line
                    self.line_detected, self.line_center_x, debug_image = self.detect_line(cv_image)
                    
                    # Publish debug image
                    if debug_image is not None:
                        debug_msg = cv2_to_imgmsg(debug_image, encoding="bgr8")
                        debug_msg.header = self.current_image.header
                        self.debug_image_pub.publish(debug_msg)
                    
                    # Publish detection status
                    self.line_detected_pub.publish(Bool(data=self.line_detected))
                    
                    # Control walking
                    self.control_walking()
            
            rate.sleep()


def main():
    try:
        node = LineFollowerV3()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 Line Follower V3 Terminated")
    except Exception as e:
        rospy.logerr(f"💥 Error in Line Follower V3: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
