#!/usr/bin/env python3
# encoding: utf-8

"""
Line Follower Node V2 - Rewritten Walking Logic
Improved stability, smoother control, better state management
"""

import rospy
import cv2
import numpy as np
import math
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from ainex_kinematics.gait_manager import GaitManager

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


class LineFollowerV2:
    """
    Improved Line Follower with cleaner walking logic
    Based on Reference visual_patrol implementation
    """
    
    def __init__(self):
        rospy.init_node('line_follower_node', anonymous=False)
        rospy.loginfo("🚀 Line Follower V2 Starting...")
        
        # ROS Parameters
        self.image_topic = rospy.get_param('~image_topic', '/camera/image_raw')
        self.control_rate = rospy.get_param('~control_rate', 10)
        self.image_process_size = rospy.get_param('~image_process_size', [160, 120])
        self.line_roi = rospy.get_param('~line_roi', [[0.5, 0.7, 0.2, 0.8]])
        self.lower_black = np.array(rospy.get_param('~lower_black', [0, 0, 0]))
        self.upper_black = np.array(rospy.get_param('~upper_black', [180, 255, 50]))
        self.min_contour_area = rospy.get_param('~min_contour_area', 50)
        
        # Walking control parameters
        self.center_tolerance = rospy.get_param('~center_tolerance', 10)  # pixels - dead zone
        self.max_turn_angle = rospy.get_param('~max_turn_angle', 8.0)  # Reference uses ±8 degrees
        self.min_turn_angle = rospy.get_param('~min_turn_angle', 4.0)  # Switch DSP at 4 degrees
        self.max_forward_speed = rospy.get_param('~forward_speed', 0.006)  # m - REDUCED for simulation
        self.min_forward_speed = rospy.get_param('~min_forward_speed', 0.004)  # m - reduced for sharp turns
        
        # Camera calibration offset (for camera alignment correction)
        self.center_x_offset = rospy.get_param('~center_x_offset', 0)  # pixels offset from geometric center
        
        # State tracking
        self.current_image = None
        self.line_detected = False
        self.line_center_x = 0
        self.last_line_x = None
        self.line_lost_count = 0
        self.max_line_lost_frames = 20  # Stop after losing line for this many frames
        self.is_initialized = False
        self.first_step_done = False
        self.rotation_direction = 1  # 1 for right, -1 for left (remembers last turn direction)
        
        # Initialize GaitManager
        rospy.loginfo("⚙️  Initializing GaitManager...")
        self.gait_manager = GaitManager()
        
        # Configure walking parameters - MUCH SLOWER for simulation stability
        # Real robot uses faster speeds, simulation needs gentler motion
        # Straight walking (go_gait_param)
        self.go_gait_param = self.gait_manager.get_gait_param()
        self.go_gait_param['body_height'] = 0.025  # m
        self.go_gait_param['step_height'] = 0.008  # m - REDUCED from 0.015 for simulation
        self.go_gait_param['hip_pitch_offset'] = 15
        self.go_gait_param['z_swap_amplitude'] = 0.004  # m - REDUCED from 0.006 for less sway
        
        # Turning walking (turn_gait_param)
        self.turn_gait_param = self.gait_manager.get_gait_param()
        self.turn_gait_param['body_height'] = 0.025  # m
        self.turn_gait_param['step_height'] = 0.010  # m - REDUCED from 0.02 for simulation
        self.turn_gait_param['hip_pitch_offset'] = 15
        self.turn_gait_param['z_swap_amplitude'] = 0.004  # m - REDUCED from 0.006
        
        # DSP (Double Support Phase) parameters: [period_ms, dsp_ratio, y_swap]
        # MUCH SLOWER for simulation - real robot uses 300/400ms
        self.go_dsp = [600, 0.3, 0.015]    # SLOW: 600ms period, longer DSP for stability
        self.turn_dsp = [700, 0.35, 0.015]  # VERY SLOW: 700ms for turning stability
        
        # Arm swing - keep at 0 for simulation
        self.go_arm_swap = 0
        self.turn_arm_swap = 0
        
        # Publishers
        self.debug_image_pub = rospy.Publisher('/line_follower_node/debug_image', Image, queue_size=1)
        self.line_detected_pub = rospy.Publisher('/line_follower_node/line_detected', Bool, queue_size=1)
        
        # Subscriber
        self.image_sub = rospy.Subscriber(
            self.image_topic, Image, self.image_callback, 
            queue_size=1, buff_size=2**24
        )
        
        rospy.loginfo("✅ Line Follower V2 Ready!")
        
    def image_callback(self, msg):
        """Store latest image"""
        self.current_image = msg
        
    def initialize_robot(self):
        """Gradual robot initialization sequence"""
        rospy.loginfo("⏳ Initializing robot - waiting for simulation to stabilize...")
        rospy.sleep(2.0)
        
        rospy.loginfo("🤖 Setting initial standing pose...")
        try:
            # Use go_gait_param for initialization
            self.gait_manager.update_pose(self.go_gait_param)
            rospy.sleep(1.5)
        except Exception as e:
            rospy.logwarn(f"Pose update warning (non-critical): {e}")
        
        rospy.loginfo("✅ Robot initialized and ready!")
        self.is_initialized = True
        
    def detect_line(self, image):
        """
        Detect black line in image using HSV color space
        Returns: (line_detected, line_center_x, debug_image)
        """
        height, width = self.image_process_size[1], self.image_process_size[0]
        resized = cv2.resize(image, (width, height))
        debug_image = resized.copy()
        
        # Convert to HSV and create mask for black color
        hsv = cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_black, self.upper_black)
        
        line_detected = False
        line_center_x = 0
        largest_contour_area = 0
        
        # Process each ROI
        for roi in self.line_roi:
            y_min = int(roi[0] * height)
            y_max = int(roi[1] * height)
            x_min = int(roi[2] * width)
            x_max = int(roi[3] * width)
            
            # Draw ROI on debug image
            cv2.rectangle(debug_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 1)
            
            # Extract ROI mask
            roi_mask = mask[y_min:y_max, x_min:x_max]
            
            # Find contours (handle OpenCV 3 vs 4)
            contour_result = cv2.findContours(roi_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contour_result) == 3:
                _, contours, _ = contour_result
            else:
                contours, _ = contour_result
            
            # Find largest valid contour
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
                        
                        # Draw contour and center point
                        adjusted_contour = largest_contour + np.array([x_min, y_min])
                        cv2.drawContours(debug_image, [adjusted_contour], -1, (0, 0, 255), 2)
                        cv2.circle(debug_image, (line_center_x, cy + y_min), 5, (255, 0, 0), -1)
        
        # Draw alignment reference lines (crosshair)
        # Geometric center (yellow)
        cv2.line(debug_image, (width // 2, 0), (width // 2, height), (0, 255, 255), 1)  # Vertical yellow line
        cv2.line(debug_image, (0, height // 2), (width, height // 2), (0, 255, 255), 1)  # Horizontal yellow line
        
        # Calibrated center (green) - the ACTUAL alignment target
        calibrated_center_x = int(width / 2.0 + self.center_x_offset)
        cv2.line(debug_image, (calibrated_center_x, 0), (calibrated_center_x, height), (0, 255, 0), 2)  # Thick green line
        
        status_text = "Line: YES" if line_detected else "Line: NO"
        status_color = (0, 255, 0) if line_detected else (0, 0, 255)
        cv2.putText(debug_image, status_text, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, status_color, 1)
        
        # Display calibration info
        if self.center_x_offset != 0:
            cv2.putText(debug_image, f"Calib: {self.center_x_offset:+d}px", (5, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        
        if line_detected:
            error = line_center_x - calibrated_center_x
            cv2.putText(debug_image, f"X: {line_center_x} Err: {error:+d}px", (5, 45 if self.center_x_offset != 0 else 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        return line_detected, line_center_x, debug_image
    
    def calculate_turn_angle(self, line_x, image_width):
        """
        Calculate turn angle based on line position - Reference implementation
        Uses 3-zone control: dead zone, proportional, and max turn
        
        Returns: turn_angle in degrees (negative = turn left, positive = turn right)
        """
        center_x = image_width / 2.0 + self.center_x_offset
        error = line_x - center_x
        
        # Zone 1: Dead zone - no turning if close to center (within 10 pixels)
        if abs(error) < self.center_tolerance:
            return 0
        
        # Zone 2: Proportional control (within width/6 from center)
        elif abs(error) < image_width / 6:
            # Reference formula: copysign(1, error) + val_map(error, -width/6, width/6, yaw_range[0]+1, yaw_range[1]-1)
            # This creates smooth proportional control with a bias of ±1 degree
            proportional = self._val_map(error, -image_width/6, image_width/6, 
                                        -self.max_turn_angle + 1, self.max_turn_angle - 1)
            return math.copysign(1, error) + proportional
        
        # Zone 3: Maximum turn (beyond width/6 from center)
        else:
            return math.copysign(self.max_turn_angle, error)
    
    def _val_map(self, value, in_min, in_max, out_min, out_max):
        """Map value from input range to output range (like Arduino map function)"""
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
    def calculate_forward_speed(self, turn_angle):
        """
        Calculate forward speed based on turn angle - Reference implementation adapted for simulation
        Slow down significantly when turning sharply (abs_turn > 6 degrees)
        
        Returns: forward_speed in meters
        """
        abs_turn = abs(turn_angle)
        
        # Adapted for simulation: slower speeds to prevent instability
        if abs_turn > 6:
            return self.min_forward_speed  # 0.004m when sharp turning
        else:
            return self.max_forward_speed  # 0.006m when going straight or gentle turn
    
    def execute_walk_command(self, forward_speed, turn_angle, is_turning):
        """
        Execute walking command using GaitManager.set_step() - Reference implementation
        Uses different gait params and DSP based on turn angle (threshold at 4 degrees)
        
        Args:
            forward_speed: meters
            turn_angle: degrees (will be negated for proper direction)
            is_turning: whether robot is turning (not used, we check abs(turn_angle) < 4)
        """
        # Reference: if abs(yaw_output) < 4: use go_gait, else: use turn_gait
        if abs(turn_angle) < 4:
            # Going straight or very gentle turn - use go_gait_param
            self.gait_manager.set_step(
                self.go_dsp,              # [300, 0.2, 0.02] - faster
                forward_speed,            # x_amplitude (meters)
                0,                        # y_amplitude (meters)
                int(-turn_angle),         # rotation_angle (degrees, negated)
                self.go_gait_param,       # straight gait parameters
                arm_swap=self.go_arm_swap,  # arm swing
                step_num=0
            )
        else:
            # Turning - use turn_gait_param
            self.gait_manager.set_step(
                self.turn_dsp,            # [400, 0.2, 0.02] - slower for stability
                forward_speed,            # x_amplitude (meters)
                0,                        # y_amplitude (meters)
                int(-turn_angle),         # rotation_angle (degrees, negated)
                self.turn_gait_param,     # turning gait parameters
                arm_swap=self.turn_arm_swap,  # arm swing
                step_num=0
            )
    
    def control_walking(self):
        """
        Main walking control logic
        When line is lost, robot moves forward in last known direction
        """
        if not self.line_detected:
            # Line not detected
            self.line_lost_count += 1
            
            if self.line_lost_count < self.max_line_lost_frames and self.last_line_x is not None:
                # Line lost - Continue moving FORWARD in the last known direction
                # Calculate the last known turn direction
                if self.line_lost_count == 1:
                    # Just lost the line - calculate last known direction
                    image_center = self.image_process_size[0] / 2.0
                    error = self.last_line_x - image_center
                    
                    # Calculate turn angle based on last position
                    if abs(error) < self.center_tolerance:
                        self.rotation_direction = 0  # Was going straight
                    elif abs(error) < self.image_process_size[0] / 6:
                        # Small offset - gentle turn
                        turn_ratio = abs(error) / (self.image_process_size[0] / 6)
                        angle = self.min_turn_angle + turn_ratio * (self.max_turn_angle - self.min_turn_angle)
                        self.rotation_direction = math.copysign(angle, error)
                    else:
                        # Large offset - use max turn
                        self.rotation_direction = math.copysign(self.max_turn_angle, error)
                
                # Move FORWARD while maintaining last known turn direction
                rospy.logwarn_throttle(0.5, f"⚠️  Line lost ({self.line_lost_count}/{self.max_line_lost_frames}) - continuing in last direction (turn: {self.rotation_direction:.1f}°)...")
                
                self.execute_walk_command(
                    forward_speed=self.min_forward_speed * 0.5,  # Slow forward movement
                    turn_angle=self.rotation_direction,  # Maintain last turn direction
                    is_turning=abs(self.rotation_direction) > 0.5  # Any turn above 0.5 degrees
                )
            else:
                # Lost line for too long - stop
                if self.first_step_done:
                    rospy.logwarn_throttle(2.0, "❌ Line lost for too long - STOPPED")
                    self.gait_manager.stop()
                else:
                    rospy.logwarn_throttle(2.0, "❌ No line detected - waiting...")
            return
        
        # Line detected - reset lost counter
        self.line_lost_count = 0
        self.last_line_x = self.line_center_x
        
        # First step - go very gently
        if not self.first_step_done:
            rospy.loginfo("🚶 Taking first step gently...")
            self.execute_walk_command(
                forward_speed=0.002,  # Very small first step - REDUCED for stability
                turn_angle=0,
                is_turning=False
            )
            self.first_step_done = True
            rospy.sleep(1.5)  # Longer pause after first step for stability
            return
        
        # Calculate control outputs
        image_width = self.image_process_size[0]
        turn_angle = self.calculate_turn_angle(self.line_center_x, image_width)
        forward_speed = self.calculate_forward_speed(turn_angle)
        is_turning = abs(turn_angle) >= self.min_turn_angle
        
        # Execute walking command
        self.execute_walk_command(forward_speed, turn_angle, is_turning)
        
        # Logging
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
                # Process image
                cv_image = imgmsg_to_cv2(self.current_image)
                
                if cv_image is not None:
                    # Detect line
                    self.line_detected, self.line_center_x, debug_image = self.detect_line(cv_image)
                    
                    # Publish debug image
                    if debug_image is not None:
                        debug_msg = cv2_to_imgmsg(debug_image, encoding="bgr8")
                        debug_msg.header = self.current_image.header
                        self.debug_image_pub.publish(debug_msg)
                    
                    # Publish line detection status
                    self.line_detected_pub.publish(Bool(data=self.line_detected))
                    
                    # Control walking
                    self.control_walking()
            
            rate.sleep()


def main():
    try:
        node = LineFollowerV2()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 Line Follower V2 Terminated")
    except Exception as e:
        rospy.logerr(f"💥 Error in Line Follower V2: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
