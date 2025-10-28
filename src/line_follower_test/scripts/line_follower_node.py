#!/usr/bin/env python3
# encoding: utf-8

"""
Line Follower Node for Ainex Humanoid Robot in Gazebo Simulation
"""

import rospy
import cv2
import numpy as np
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

class LineFollowerNode:
    def __init__(self):
        rospy.init_node('line_follower_node', anonymous=False)
        rospy.loginfo("Line Follower Node Starting...")
        
        self.image_topic = rospy.get_param('~image_topic', '/camera/image_raw')
        self.control_rate = rospy.get_param('~control_rate', 10)
        self.image_process_size = rospy.get_param('~image_process_size', [160, 120])
        self.line_roi = rospy.get_param('~line_roi', [[0.5, 0.7, 0.2, 0.8]])
        self.lower_black = np.array(rospy.get_param('~lower_black', [0, 0, 0]))
        self.upper_black = np.array(rospy.get_param('~upper_black', [180, 255, 50]))
        self.min_contour_area = rospy.get_param('~min_contour_area', 50)
        self.forward_speed = rospy.get_param('~forward_speed', 0.01)
        self.turn_gain = rospy.get_param('~turn_gain', 0.5)
        self.max_turn_angle = rospy.get_param('~max_turn_angle', 8.0)
        self.walking_speed = rospy.get_param('~walking_speed', 4)
        self.body_height = rospy.get_param('~body_height', 0.025)
        
        self.current_image = None
        self.line_detected = False
        self.line_center_x = 0
        self.last_line_x = None
        self.first_move = True  # Track first movement to make it gradual
        
        # Initialize GaitManager (this is the proper way!)
        rospy.loginfo("Initializing GaitManager...")
        self.gait_manager = GaitManager()
        
        # Get and configure walking parameters
        self.walking_param = self.gait_manager.get_gait_param()
        self.walking_param['body_height'] = self.body_height
        self.walking_param['step_height'] = 0.010  # REDUCED from 0.015 for stability
        self.walking_param['hip_pitch_offset'] = 15
        self.walking_param['z_swap_amplitude'] = 0.005  # REDUCED from 0.006 for stability
        
        self.debug_image_pub = rospy.Publisher('/line_follower_node/debug_image', Image, queue_size=1)
        self.line_detected_pub = rospy.Publisher('/line_follower_node/line_detected', Bool, queue_size=1)
        
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1, buff_size=2**24)
        
        # Add startup state tracking
        self.startup_time = rospy.Time.now()
        self.is_initialized = False
        
        rospy.loginfo("Line Follower Node Ready!")
        
    def image_callback(self, msg):
        self.current_image = msg
        
    def process_image(self):
        rate = rospy.Rate(self.control_rate)
        
        # Wait for simulation to stabilize (Gazebo already paused during spawn)
        rospy.loginfo("⏳ Waiting 2 seconds for camera and topics to be ready...")
        rospy.sleep(2.0)
        
        # Gradually transition to standing pose using update_pose
        rospy.loginfo("🤖 Gradually transitioning to standing posture...")
        try:
            self.gait_manager.update_pose(self.walking_param)
            rospy.sleep(1.0)  # Give time for pose update
        except Exception as e:
            rospy.logwarn(f"Pose update issue (non-critical): {e}")
        
        rospy.loginfo("✅ Robot stabilized and ready to follow line!")
        self.is_initialized = True
        
        while not rospy.is_shutdown():
            if self.current_image is not None:
                cv_image = imgmsg_to_cv2(self.current_image)
                if cv_image is not None:
                    self.line_detected, self.line_center_x, debug_image = self.detect_line(cv_image)
                    if debug_image is not None:
                        debug_msg = cv2_to_imgmsg(debug_image, encoding="bgr8")
                        debug_msg.header = self.current_image.header
                        self.debug_image_pub.publish(debug_msg)
                    self.line_detected_pub.publish(Bool(data=self.line_detected))
                    self.control_walking()
            rate.sleep()
    
    def detect_line(self, image):
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
            
            # OpenCV 3 returns (image, contours, hierarchy), OpenCV 4 returns (contours, hierarchy)
            contour_result = cv2.findContours(roi_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if len(contour_result) == 3:
                _, contours, _ = contour_result  # OpenCV 3
            else:
                contours, _ = contour_result  # OpenCV 4
            
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
        status_text = "Line Detected: YES" if line_detected else "Line Detected: NO"
        cv2.putText(debug_image, status_text, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0) if line_detected else (0, 0, 255), 1)
        if line_detected:
            cv2.putText(debug_image, f"X: {line_center_x}", (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        return line_detected, line_center_x, debug_image
    
    def control_walking(self):
        """Use GaitManager.move() for continuous walking based on line detection"""
        if self.line_detected:
            # Calculate turn angle based on line position
            image_center_x = self.image_process_size[0] / 2.0
            error = (self.line_center_x - image_center_x) / image_center_x
            turn_angle = error * self.turn_gain * self.max_turn_angle
            turn_angle = max(-self.max_turn_angle, min(self.max_turn_angle, turn_angle))
            
            # Convert to integer degrees for GaitManager
            turn_angle_int = int(round(turn_angle))
            
            # Gradual start: use slower speed for first movement to avoid jerk
            if self.first_move:
                rospy.loginfo("🚶 Starting walking motion gradually...")
                # Start with VERY slow first step - use slowest speed (4)
                self.gait_manager.move(
                    4,  # Speed 4 = SLOWEST (600ms period)
                    0.003,  # VERY small forward amplitude
                    0.0,
                    0,  # No turning on first move
                    arm_swap=0,
                    step_num=0
                )
                self.first_move = False
                rospy.sleep(1.5)  # Even longer pause after first command
                return
            
            # Use GaitManager.move() - simpler API for continuous walking
            # Parameters: speed (1-4), x, y, rotation_angle, arm_swap, step_num
            self.gait_manager.move(
                self.walking_speed,  # Speed 1-4 (4 is fastest)
                self.forward_speed,  # x_amplitude (meters)
                0.0,                 # y_amplitude (meters)
                turn_angle_int,      # rotation_angle (degrees)
                arm_swap=0,          # No arm swing for stability
                step_num=0           # Continuous walking
            )
            
            rospy.loginfo_throttle(1.0, f"✅ Following line - X: {self.line_center_x}, Error: {error:.2f}, Turn: {turn_angle:.1f}°")
            self.last_line_x = self.line_center_x
        else:
            # Line not detected - keep moving forward slowly to try to find it
            if self.last_line_x is not None:
                rospy.logwarn_throttle(2.0, "⚠️ Line lost - continuing forward slowly...")
                self.gait_manager.move(
                    4,  # Force slowest speed (4 = 600ms period)
                    self.forward_speed * 0.3,  # Very slow (reduced from 0.5)
                    0.0,
                    0,  # No turning
                    arm_swap=0,
                    step_num=0
                )
            else:
                # Never seen line - don't move
                if not self.first_move:
                    # Only stop if we were moving before
                    self.gait_manager.stop()
                rospy.logwarn_throttle(2.0, "❌ Line never detected - STOPPED")


def main():
    try:
        node = LineFollowerNode()
        node.process_image()
    except rospy.ROSInterruptException:
        rospy.loginfo("Line Follower Node Terminated")
    except Exception as e:
        rospy.logerr(f"Error in Line Follower Node: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()
