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
        
        # COMMON ROI for both line following and obstacle detection
        # Format: [y_min, y_max, x_min, x_max] as fractions of image size
        # Expanded ROI to cover both lower (line) and upper (obstacles) regions
        self.common_roi = rospy.get_param('~common_roi', [0.3, 0.8, 0.1, 0.9])
        
        # Line detection parameters
        self.lower_black = np.array(rospy.get_param('~lower_black', [0, 0, 0]))
        self.upper_black = np.array(rospy.get_param('~upper_black', [180, 255, 50]))
        self.min_contour_area = rospy.get_param('~min_contour_area', 50)
        
        # Color detection for obstacles (stairs=red, hurdle=blue)
        # Red color detection (HSV) - need two ranges because red wraps around 0/180
        self.lower_red1 = np.array([0, 100, 100])      # Lower red range
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([160, 100, 100])    # Upper red range
        self.upper_red2 = np.array([180, 255, 255])
        
        # Blue color detection (HSV)
        self.lower_blue = np.array([100, 100, 100])
        self.upper_blue = np.array([130, 255, 255])
        
        # Minimum area for obstacle detection
        self.min_obstacle_area = rospy.get_param('~min_obstacle_area', 100)
        
        # Walking control parameters
        self.center_tolerance = rospy.get_param('~center_tolerance', 10)  # pixels - dead zone
        self.max_turn_angle = rospy.get_param('~max_turn_angle', 10.0)  # Lesson 4 default
        self.min_turn_angle = rospy.get_param('~min_turn_angle', 5.0)  # Switch DSP threshold
        self.max_forward_speed = rospy.get_param('~forward_speed', 0.05)  # Tutorial default: 0.05m
        self.min_forward_speed = rospy.get_param('~min_forward_speed', 0.02)  # reduced for sharp turns
        
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
        
        # Obstacle color detection state
        self.detected_colors = []  # List of detected obstacle colors
        self.last_obstacle_detection = None  # Last detected obstacle type
        
        # Initialize GaitManager
        rospy.loginfo("⚙️  Initializing GaitManager...")
        self.gait_manager = GaitManager()
        
        # Configure walking parameters - EXACT defaults from Lesson 4 tutorial
        # Source: Reference/ainex_driver/ainex_kinematics/config/walking_param_sim.yaml
        
        # Straight walking (go_gait_param)
        self.go_gait_param = self.gait_manager.get_gait_param()
        self.go_gait_param['body_height'] = 0.025          # init_z_offset
        self.go_gait_param['step_height'] = 0.015          # z_move_amplitude
        self.go_gait_param['hip_pitch_offset'] = 15        # hip_pitch_offset
        self.go_gait_param['z_swap_amplitude'] = 0.006     # z_swap_amplitude
        self.go_gait_param['pelvis_offset'] = 0            # pelvis_offset
        
        # Turning walking (turn_gait_param)
        self.turn_gait_param = self.gait_manager.get_gait_param()
        self.turn_gait_param['body_height'] = 0.025
        self.turn_gait_param['step_height'] = 0.020        # Slightly higher for turns
        self.turn_gait_param['hip_pitch_offset'] = 15
        self.turn_gait_param['z_swap_amplitude'] = 0.006
        self.turn_gait_param['pelvis_offset'] = 0
        
        # DSP (Double Support Phase) parameters: [period_ms, dsp_ratio, y_swap_amplitude]
        # EXACT simulation defaults from walking_param_sim.yaml
        self.go_dsp = [1500, 0.3, 0.035]    # Lesson 4: 1500ms period for simulation
        self.turn_dsp = [1800, 0.35, 0.035]  # Slower for turning stability
        
        # Arm swing - disabled in simulation (as per walking_param_sim.yaml)
        self.go_arm_swap = 0  # arm_swing_gain: 0.0
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
        Uses common ROI shared with obstacle detection
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
        
        # Use common ROI for line detection
        roi = self.common_roi
        y_min = int(roi[0] * height)
        y_max = int(roi[1] * height)
        x_min = int(roi[2] * width)
        x_max = int(roi[3] * width)
        
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
            
            if contour_area > self.min_contour_area:
                largest_contour_area = contour_area
                M = cv2.moments(largest_contour)
                
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    line_center_x = cx + x_min
                    line_detected = True
                    
                    # Draw bounding box around the detected line (white box)
                    x, y, w, h = cv2.boundingRect(largest_contour)
                    cv2.rectangle(debug_image, (x_min + x, y_min + y), 
                                (x_min + x + w, y_min + y + h), (255, 255, 255), 2)
        
        return line_detected, line_center_x, debug_image
    
    def detect_obstacles(self, image, debug_image):
        """
        Detect colored obstacles (stairs=red, hurdle=blue) in the image
        Uses the same common ROI as line detection
        Updates debug_image with colored bounding boxes and labels
        Returns: list of detected colors ['RED', 'BLUE', etc.]
        """
        height, width = self.image_process_size[1], self.image_process_size[0]
        resized = cv2.resize(image, (width, height))
        
        # Convert to HSV
        hsv = cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)
        
        detected_colors = []
        
        # Use common ROI (same as line detection)
        roi = self.common_roi
        y_min = int(roi[0] * height)
        y_max = int(roi[1] * height)
        x_min = int(roi[2] * width)
        x_max = int(roi[3] * width)
        
        # NOTE: ROI box is already drawn by detect_line() method
        # No need to draw it again here
        
        # Extract ROI
        hsv_roi = hsv[y_min:y_max, x_min:x_max]
        
        # Detect RED (stairs) - combine two red ranges
        mask_red1 = cv2.inRange(hsv_roi, self.lower_red1, self.upper_red1)
        mask_red2 = cv2.inRange(hsv_roi, self.lower_red2, self.upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        
        # Find red contours
        contour_result = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contour_result) == 3:
            _, contours_red, _ = contour_result
        else:
            contours_red, _ = contour_result
        
        # Process ALL red contours (stairs) - detect multiple instances
        red_count = 0
        for contour in contours_red:
            red_area = cv2.contourArea(contour)
            
            if red_area > self.min_obstacle_area:
                if 'RED' not in detected_colors:
                    detected_colors.append('RED')
                red_count += 1
                
                # Draw red bounding box around detected red obstacle
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(debug_image, (x_min + x, y_min + y), 
                            (x_min + x + w, y_min + y + h), (0, 0, 255), 2)
        
        # Detect BLUE (hurdle)
        mask_blue = cv2.inRange(hsv_roi, self.lower_blue, self.upper_blue)
        
        # Find blue contours
        contour_result = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contour_result) == 3:
            _, contours_blue, _ = contour_result
        else:
            contours_blue, _ = contour_result
        
        # Process ALL blue contours (hurdle) - detect multiple instances
        blue_count = 0
        for contour in contours_blue:
            blue_area = cv2.contourArea(contour)
            
            if blue_area > self.min_obstacle_area:
                if 'BLUE' not in detected_colors:
                    detected_colors.append('BLUE')
                blue_count += 1
                
                # Draw blue bounding box around detected blue obstacle
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(debug_image, (x_min + x, y_min + y), 
                            (x_min + x + w, y_min + y + h), (255, 0, 0), 2)
        
        # MINIMAL UI: Display only color detection text (REDUCED SIZE)
        if detected_colors:
            color_text = "Color: "
            if 'RED' in detected_colors and 'BLUE' in detected_colors:
                color_text += "Red + Blue"
            elif 'RED' in detected_colors:
                color_text += "Red"
            elif 'BLUE' in detected_colors:
                color_text += "Blue"
            
            # Draw text with black background for better visibility (smaller font)
            text_size = cv2.getTextSize(color_text, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)[0]
            cv2.rectangle(debug_image, (5, height - text_size[1] - 10),
                        (text_size[0] + 10, height - 5), (0, 0, 0), -1)
            cv2.putText(debug_image, color_text, (8, height - 8),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        return detected_colors
    
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
        Calculate forward speed based on turn angle - Reference implementation
        ALWAYS moves forward, even when turning sharply
        Reference: if abs(yaw) > 6: x_output = 0.008, else: x_output = self.x_max
        
        Returns: forward_speed in meters
        """
        abs_turn = abs(turn_angle)
        
        # CRITICAL: ALWAYS move forward (Reference behavior)
        # When turning sharply (> 6 degrees), reduce speed but still move forward
        if abs_turn > 6:
            return 0.008  # Reference uses 0.008m for sharp turns (still moving forward!)
        else:
            return self.max_forward_speed  # Full speed when straight or gentle turn
    
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
        Based on Reference: Only send commands when line is detected
        When line is lost, robot continues with last command (keeps walking)
        """
        if not self.line_detected:
            # Line not detected - DON'T send new commands
            # Robot will continue executing the last walking command
            self.line_lost_count += 1
            
            # Only log occasionally to avoid spam
            if self.line_lost_count == 1:
                rospy.logwarn(f"⚠️  Line lost - robot continues with last command...")
            elif self.line_lost_count % 20 == 0:
                rospy.logwarn(f"⚠️  Line still lost ({self.line_lost_count} frames) - still searching...")
            
            # CRITICAL: Don't send any new commands - let robot keep executing last command
            # This allows it to continue moving and potentially find the line again
            return
        
        # Line detected - reset lost counter
        if self.line_lost_count > 0:
            rospy.loginfo(f"✅ Line found again after {self.line_lost_count} frames!")
        
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
                    
                    # Detect obstacles (stairs and hurdles)
                    self.detected_colors = self.detect_obstacles(cv_image, debug_image)
                    
                    # Log obstacle detection to terminal
                    if self.detected_colors:
                        # Only log when detection changes to avoid spam
                        current_detection = ",".join(sorted(self.detected_colors))
                        if current_detection != self.last_obstacle_detection:
                            obstacle_names = []
                            if 'RED' in self.detected_colors:
                                obstacle_names.append("🔴 STAIRS (RED)")
                            if 'BLUE' in self.detected_colors:
                                obstacle_names.append("🔵 HURDLE (BLUE)")
                            
                            rospy.loginfo(f"🚧 OBSTACLE DETECTED: {' + '.join(obstacle_names)}")
                            self.last_obstacle_detection = current_detection
                    else:
                        if self.last_obstacle_detection is not None:
                            rospy.loginfo("✅ No obstacles detected")
                            self.last_obstacle_detection = None
                    
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
