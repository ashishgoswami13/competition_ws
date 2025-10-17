#!/usr/bin/env python3#!/usr/bin/env python3#!/usr/bin/env python3#!/usr/bin/env python3

# encoding: utf-8

# encoding: utf-8

"""

Line Follower Node for Ainex Humanoid Robot in Gazebo Simulation"""# encoding: utf-8# encoding: utf-8



This node processes camera images to detect a line and controls the robot to follow it.Line Follower Node for Ainex Humanoid Robot in Gazebo Simulation

Uses ainex_controller for walking control.

"""This node processes camera images to detect a line and controls the robot to follow it.""""""



import rospyUses ainex_controller for walking control.

import cv2

import numpy as np"""Line Follower Node for Ainex Humanoid Robot in Gazebo SimulationLine Follower Node for Ainex Humanoid Robot in Gazebo Simulation

from sensor_msgs.msg import Image

from std_msgs.msg import Bool, String

from ainex_interfaces.msg import AppWalkingParam, AppAction

import rospyThis node processes camera images to detect a line and controls the robot to follow it.This node processes camera images to detect a line and controls the robot to follow it.

def imgmsg_to_cv2(img_msg):

    """import cv2

    Convert ROS Image message to OpenCV image (BGR8).

    Manual conversion to avoid cv_bridge Python 2/3 compatibility issues.import numpy as npUses ainex_controller for walking control.Adapted from the real-world visual_patrol_node.py for simulation use.

    """

    if img_msg.encoding == "rgb8":from sensor_msgs.msg import Image

        # RGB8: 3 channels, uint8

        img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, 3)from std_msgs.msg import String, Bool""""""

        # Convert RGB to BGR for OpenCV

        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)from ainex_interfaces.msg import AppWalkingParam

    elif img_msg.encoding == "bgr8":

        # BGR8: 3 channels, uint8

        img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, 3)

    elif img_msg.encoding == "mono8":# Manual image conversion to avoid cv_bridge Python 2/3 issues

        # Mono8: 1 channel, uint8

        img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width)def imgmsg_to_cv2(img_msg):import rospyimport rospy

    elif img_msg.encoding == "32FC1":

        # Float 1 channel    """

        img = np.frombuffer(img_msg.data, dtype=np.float32).reshape(img_msg.height, img_msg.width)

    else:    Convert ROS Image message to OpenCV image (numpy array).import cv2import cv2

        rospy.logerr(f"Unsupported encoding: {img_msg.encoding}")

        return None    Handles RGB8 and BGR8 encodings manually to avoid cv_bridge issues.

    

    return img    """import numpy as npimport numpy as np



def cv2_to_imgmsg(cv_image, encoding="bgr8"):    if img_msg.encoding == "rgb8" or img_msg.encoding == "bgr8":

    """

    Convert OpenCV image to ROS Image message.        dtype = np.uint8from sensor_msgs.msg import Imagefrom sensor_msgs.msg import Image

    Manual conversion to avoid cv_bridge Python 2/3 compatibility issues.

    """        n_channels = 3

    img_msg = Image()

    img_msg.height = cv_image.shape[0]    elif img_msg.encoding == "mono8":from std_msgs.msg import String, Boolfrom std_msgs.msg import Float64, Bool

    img_msg.width = cv_image.shape[1]

    img_msg.encoding = encoding        dtype = np.uint8

    img_msg.step = cv_image.shape[1] * cv_image.shape[2] if len(cv_image.shape) > 2 else cv_image.shape[1]

    img_msg.data = cv_image.tobytes()        n_channels = 1from ainex_interfaces.msg import AppWalkingParamimport math

    return img_msg

    else:



class LineFollowerNode:        rospy.logerr("Unsupported encoding: {}".format(img_msg.encoding))

    def __init__(self):

        rospy.init_node('line_follower_node', anonymous=False)        return None

        rospy.loginfo("Line Follower Node Starting...")

            # Manual image conversion to avoid cv_bridge Python 2/3 issues# Manual image conversion to avoid cv_bridge Python 2/3 issues

        # Get parameters

        self.image_topic = rospy.get_param('~image_topic', '/camera/image_raw')    # Convert image data bytes to numpy array

        self.control_rate = rospy.get_param('~control_rate', 10)

            img_buf = np.frombuffer(img_msg.data, dtype=dtype)def imgmsg_to_cv2(img_msg):def imgmsg_to_cv2(img_msg):

        # Line detection parameters from config

        self.image_process_size = rospy.get_param('~image_process_size', [160, 120])    

        self.line_roi = rospy.get_param('~line_roi', [[0.5, 0.7, 0.2, 0.8]])  # [y_min, y_max, x_min, x_max]

        self.lower_black = np.array(rospy.get_param('~lower_black', [0, 0, 0]))    if n_channels == 1:    """    """

        self.upper_black = np.array(rospy.get_param('~upper_black', [180, 255, 50]))

        self.min_contour_area = rospy.get_param('~min_contour_area', 50)        cv_image = img_buf.reshape(img_msg.height, img_msg.width)

        

        # Walking control parameters    else:    Convert ROS Image message to OpenCV image (numpy array).    Convert ROS Image message to OpenCV image (numpy array).

        self.forward_speed = rospy.get_param('~forward_speed', 0.01)  # meters per step

        self.turn_gain = rospy.get_param('~turn_gain', 0.5)        cv_image = img_buf.reshape(img_msg.height, img_msg.width, n_channels)

        self.max_turn_angle = rospy.get_param('~max_turn_angle', 8.0)  # degrees

        self.walking_speed = rospy.get_param('~walking_speed', 4)  # 1-4 (fastest)        Handles RGB8 and BGR8 encodings manually to avoid cv_bridge issues.    Handles RGB8 and BGR8 encodings manually to avoid cv_bridge issues.

        self.body_height = rospy.get_param('~body_height', 0.025)  # meters

            # If RGB, convert to BGR for OpenCV

        # State variables

        self.current_image = None    if img_msg.encoding == "rgb8":    """    """

        self.line_detected = False

        self.line_center_x = 0        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

        self.last_line_x = None

                if img_msg.encoding == "rgb8" or img_msg.encoding == "bgr8":    if img_msg.encoding == "rgb8" or img_msg.encoding == "bgr8":

        # Publishers

        self.debug_image_pub = rospy.Publisher('/line_follower_node/debug_image', Image, queue_size=1)    return cv_image

        self.line_detected_pub = rospy.Publisher('/line_follower_node/line_detected', Bool, queue_size=1)

        self.walking_param_pub = rospy.Publisher('/app/set_walking_param', AppWalkingParam, queue_size=1)        dtype = np.uint8        dtype = np.uint8

        self.action_pub = rospy.Publisher('/app/set_action', AppAction, queue_size=1)

        def cv2_to_imgmsg(cv_image, encoding="bgr8"):

        # Subscriber

        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1, buff_size=2**24)    """        n_channels = 3        n_channels = 3

        

        rospy.loginfo(f"Subscribed to: {self.image_topic}")    Convert OpenCV image (numpy array) to ROS Image message.

        rospy.loginfo(f"Control rate: {self.control_rate} Hz")

        rospy.loginfo(f"Image process size: {self.image_process_size}")    """    elif img_msg.encoding == "mono8":    elif img_msg.encoding == "mono8":

        rospy.loginfo(f"Line ROI: {self.line_roi}")

        rospy.loginfo(f"Forward speed: {self.forward_speed} m")    img_msg = Image()

        rospy.loginfo(f"Turn gain: {self.turn_gain}")

        rospy.loginfo(f"Max turn angle: {self.max_turn_angle} deg")    img_msg.height, img_msg.width = cv_image.shape[0:2]        dtype = np.uint8        dtype = np.uint8

        rospy.loginfo("Line Follower Node Ready!")

            img_msg.encoding = encoding

        # Give controller time to initialize

        rospy.sleep(2.0)    if len(cv_image.shape) == 2:        n_channels = 1        n_channels = 1

        

        # Start action (stand ready)        img_msg.step = img_msg.width

        self.send_action("start")

        rospy.sleep(1.0)    else:    else:    else:

        

    def send_action(self, action_name):        img_msg.step = img_msg.width * cv_image.shape[2]

        """Send action command to ainex_controller"""

        msg = AppAction()    img_msg.data = cv_image.tobytes()        rospy.logerr("Unsupported encoding: {}".format(img_msg.encoding))        rospy.logerr("Unsupported encoding: {}".format(img_msg.encoding))

        msg.action = action_name

        self.action_pub.publish(msg)    return img_msg

        rospy.loginfo(f"Action sent: {action_name}")

                return None        return None

    def image_callback(self, msg):

        """Store the latest image"""

        self.current_image = msg

        class LineFollowerNode:        

    def process_image(self):

        """Main image processing and control loop"""    def __init__(self):

        rate = rospy.Rate(self.control_rate)

                rospy.init_node('line_follower_node', anonymous=False)    # Convert image data bytes to numpy array    # Convert image data bytes to numpy array

        while not rospy.is_shutdown():

            if self.current_image is not None:        

                # Convert ROS image to OpenCV

                cv_image = imgmsg_to_cv2(self.current_image)        # Get parameters    img_buf = np.frombuffer(img_msg.data, dtype=dtype)    # img_msg.data is a list/bytes, convert to numpy array properly

                

                if cv_image is not None:        image_topic = rospy.get_param('~image_topic', '/camera/image_raw')

                    # Detect line

                    self.line_detected, self.line_center_x, debug_image = self.detect_line(cv_image)        self.image_process_size = rospy.get_param('~image_process_size', [160, 120])        img_buf = np.frombuffer(img_msg.data, dtype=dtype)

                    

                    # Publish debug image        self.line_roi = rospy.get_param('~line_roi', [

                    if debug_image is not None:

                        debug_msg = cv2_to_imgmsg(debug_image, encoding="bgr8")            [0.5, 0.7, 0.2, 0.8]  # Single ROI in lower portion of image    if n_channels == 1:    

                        debug_msg.header = self.current_image.header

                        self.debug_image_pub.publish(debug_msg)        ])

                    

                    # Publish line detected status                cv_image = img_buf.reshape(img_msg.height, img_msg.width)    if n_channels == 1:

                    self.line_detected_pub.publish(Bool(data=self.line_detected))

                            # Color detection thresholds (HSV) for black line

                    # Control walking based on line detection

                    self.control_walking()        lower_black = rospy.get_param('~lower_black', [0, 0, 0])    else:        cv_image = img_buf.reshape(img_msg.height, img_msg.width)

            

            rate.sleep()        upper_black = rospy.get_param('~upper_black', [180, 255, 50])

    

    def detect_line(self, image):        self.lower_black = np.array(lower_black, dtype=np.uint8)        cv_image = img_buf.reshape(img_msg.height, img_msg.width, n_channels)    else:

        """

        Detect black line in the image        self.upper_black = np.array(upper_black, dtype=np.uint8)

        Returns: (line_detected, line_center_x, debug_image)

        """                    cv_image = img_buf.reshape(img_msg.height, img_msg.width, n_channels)

        # Resize image for faster processing

        height, width = self.image_process_size[1], self.image_process_size[0]        # Walking parameters

        resized = cv2.resize(image, (width, height))

        debug_image = resized.copy()        self.forward_speed = rospy.get_param('~forward_speed', 0.01)  # m forward    # If RGB, convert to BGR for OpenCV    

        

        # Convert to HSV        self.turn_gain = rospy.get_param('~turn_gain', 0.5)  # Turning sensitivity

        hsv = cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)

                self.max_turn_angle = rospy.get_param('~max_turn_angle', 8.0)  # degrees    if img_msg.encoding == "rgb8":    # If RGB, convert to BGR for OpenCV

        # Create mask for black color

        mask = cv2.inRange(hsv, self.lower_black, self.upper_black)        self.walking_speed = rospy.get_param('~walking_speed', 4)  # 1-4

        

        # Process each ROI        self.body_height = rospy.get_param('~body_height', 0.025)  # meters        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)    if img_msg.encoding == "rgb8":

        line_detected = False

        line_center_x = 0        

        largest_contour_area = 0

                # State            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

        for roi in self.line_roi:

            # ROI coordinates [y_min, y_max, x_min, x_max] in normalized [0-1]        self.line_detected = False

            y_min = int(roi[0] * height)

            y_max = int(roi[1] * height)        self.line_x = 0    return cv_image    

            x_min = int(roi[2] * width)

            x_max = int(roi[3] * width)        self.image_width = self.image_process_size[0]

            

            # Draw ROI on debug image        self.enabled = True    return cv_image

            cv2.rectangle(debug_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 1)

                    

            # Extract ROI from mask

            roi_mask = mask[y_min:y_max, x_min:x_max]        # Publishersdef cv2_to_imgmsg(cv_image, encoding="bgr8"):

            

            # Find contours        self.debug_image_pub = rospy.Publisher('/line_follower_node/debug_image', Image, queue_size=1)

            contours, _ = cv2.findContours(roi_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                    self.line_detected_pub = rospy.Publisher('/line_follower_node/line_detected', Bool, queue_size=1)    """def cv2_to_imgmsg(cv_image, encoding="bgr8"):

            if contours:

                # Find largest contour        self.walking_param_pub = rospy.Publisher('/app/set_walking_param', AppWalkingParam, queue_size=1)

                largest_contour = max(contours, key=cv2.contourArea)

                contour_area = cv2.contourArea(largest_contour)        self.walking_command_pub = rospy.Publisher('/app/set_action', String, queue_size=1)    Convert OpenCV image (numpy array) to ROS Image message.    """

                

                if contour_area > self.min_contour_area and contour_area > largest_contour_area:        

                    largest_contour_area = contour_area

                            # Subscribers    """    Convert OpenCV image (numpy array) to ROS Image message.

                    # Calculate moments to find center

                    M = cv2.moments(largest_contour)        rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=1)

                    if M["m00"] > 0:

                        cx = int(M["m10"] / M["m00"])            img_msg = Image()    """

                        cy = int(M["m01"] / M["m00"])

                                # Wait for ainex_controller to be ready

                        # Convert to full image coordinates

                        line_center_x = cx + x_min        rospy.loginfo("Waiting for ainex_controller to initialize...")    img_msg.height, img_msg.width = cv_image.shape[0:2]    img_msg = Image()

                        line_detected = True

                                rospy.sleep(3.0)

                        # Draw contour and center on debug image

                        # Adjust contour coordinates to ROI position            img_msg.encoding = encoding    img_msg.height = cv_image.shape[0]

                        adjusted_contour = largest_contour + np.array([x_min, y_min])

                        cv2.drawContours(debug_image, [adjusted_contour], -1, (0, 0, 255), 2)        # Enable walking control

                        cv2.circle(debug_image, (line_center_x, cy + y_min), 5, (255, 0, 0), -1)

                rospy.loginfo("Enabling walking controller...")    if len(cv_image.shape) == 2:    img_msg.width = cv_image.shape[1]

        # Draw center line

        cv2.line(debug_image, (width // 2, 0), (width // 2, height), (0, 255, 255), 1)        cmd_msg = String()

        

        # Add text        cmd_msg.data = "enable_control"        img_msg.step = img_msg.width    img_msg.encoding = encoding

        status_text = "Line Detected: YES" if line_detected else "Line Detected: NO"

        cv2.putText(debug_image, status_text, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0) if line_detected else (0, 0, 255), 1)        self.walking_command_pub.publish(cmd_msg)

        

        if line_detected:        rospy.sleep(0.5)    else:    img_msg.is_bigendian = 0

            cv2.putText(debug_image, f"X: {line_center_x}", (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

                

        return line_detected, line_center_x, debug_image

            cmd_msg.data = "start"        img_msg.step = img_msg.width * cv_image.shape[2]    

    def control_walking(self):

        """        self.walking_command_pub.publish(cmd_msg)

        Control robot walking based on line detection

        Uses ainex_controller's walking module        rospy.sleep(0.5)    img_msg.data = cv_image.tobytes()    if len(cv_image.shape) == 2:

        """

        walking_msg = AppWalkingParam()        

        walking_msg.speed = self.walking_speed

        walking_msg.height = self.body_height        rospy.loginfo("Line follower node initialized and ready!")    return img_msg        # Grayscale

        

        if self.line_detected:        

            # Calculate error (normalized -1 to 1)

            image_center_x = self.image_process_size[0] / 2.0    def image_callback(self, msg):        img_msg.step = img_msg.width

            error = (self.line_center_x - image_center_x) / image_center_x

                    """Process incoming camera images to detect the line."""

            # Calculate turn angle (proportional control)

            turn_angle = -error * self.turn_gain * self.max_turn_angle        try:        n_channels = 1

            

            # Clamp turn angle            # Convert ROS Image message to OpenCV image (manual conversion)

            turn_angle = max(-self.max_turn_angle, min(self.max_turn_angle, turn_angle))

                        cv_image = imgmsg_to_cv2(msg)class LineFollowerNode:    else:

            # Set walking parameters

            walking_msg.x = self.forward_speed  # Move forward            if cv_image is None:

            walking_msg.y = 0.0  # No sideways movement

            walking_msg.angle = turn_angle  # Turn angle in degrees                return    def __init__(self):        # Color

            

            rospy.loginfo_throttle(1.0, f"Following line - X: {self.line_center_x}, Error: {error:.2f}, Turn: {turn_angle:.1f}°")        except Exception as e:

            

            # Update last known position            rospy.logerr("Image conversion error: {}".format(e))        rospy.init_node('line_follower_node', anonymous=False)        n_channels = cv_image.shape[2]

            self.last_line_x = self.line_center_x

                        return

        else:

            # Line lost - stop                        img_msg.step = img_msg.width * n_channels

            walking_msg.x = 0.0

            walking_msg.y = 0.0        # Resize image for processing

            walking_msg.angle = 0.0

            rospy.logwarn_throttle(2.0, "Line not detected - Stopping")        resized = cv2.resize(cv_image, tuple(self.image_process_size))        # Get parameters    

        

        # Publish walking command        

        self.walking_param_pub.publish(walking_msg)

        # Detect line        image_topic = rospy.get_param('~image_topic', '/camera/image_raw')    img_msg.data = cv_image.tobytes()



def main():        line_detected, line_x, debug_image = self.detect_line(resized)

    try:

        node = LineFollowerNode()                self.image_process_size = rospy.get_param('~image_process_size', [160, 120])    return img_msg

        node.process_image()

    except rospy.ROSInterruptException:        # Update state

        rospy.loginfo("Line Follower Node Terminated")

    except Exception as e:        self.line_detected = line_detected        self.line_roi = rospy.get_param('~line_roi', [

        rospy.logerr(f"Error in Line Follower Node: {e}")

        import traceback        if line_detected:

        traceback.print_exc()

            self.line_x = line_x            [0.5, 0.7, 0.2, 0.8]  # Single ROI in lower portion of image



if __name__ == '__main__':            self.image_width = self.image_process_size[0]

    main()

                ])class LineFollowerController:

        # Publish debug image

        if self.debug_image_pub.get_num_connections() > 0:            """

            try:

                debug_msg = cv2_to_imgmsg(debug_image, "bgr8")        # Color detection thresholds (HSV) for black line    Simplified line following controller for simulation.

                debug_msg.header.stamp = rospy.Time.now()

                debug_msg.header.frame_id = "camera_link"        lower_black = rospy.get_param('~lower_black', [0, 0, 0])    Based on the VisualPatrol class from Reference package but adapted for Gazebo.

                self.debug_image_pub.publish(debug_msg)

            except Exception as e:        upper_black = rospy.get_param('~upper_black', [180, 255, 50])    """

                rospy.logerr("Debug image publish error: {}".format(e))

                self.lower_black = np.array(lower_black, dtype=np.uint8)    

        # Publish detection status

        self.line_detected_pub.publish(Bool(line_detected))        self.upper_black = np.array(upper_black, dtype=np.uint8)    def __init__(self):

        

        # Send walking command based on line detection                # Get parameters from parameter server

        self.control_walking()

            # Walking parameters        self.x_range = rospy.get_param('~x_range', [0, 0.010])

    def detect_line(self, image):

        """        self.forward_speed = rospy.get_param('~forward_speed', 0.01)  # m forward        self.yaw_range = rospy.get_param('~yaw_range', [-8, 8])

        Detect black line in the image using HSV color space.

                self.turn_gain = rospy.get_param('~turn_gain', 0.5)  # Turning sensitivity        self.center_x_offset = rospy.get_param('~center_x_offset', 0)

        Args:

            image: Input image (BGR format)        self.max_turn_angle = rospy.get_param('~max_turn_angle', 8.0)  # degrees        

            

        Returns:        self.walking_speed = rospy.get_param('~walking_speed', 4)  # 1-4        # Gait parameters for walking

            tuple: (line_detected, line_x, debug_image)

        """        self.body_height = rospy.get_param('~body_height', 0.025)  # meters        self.period_time = rospy.get_param('~period_time', 400)

        # Convert to HSV

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)                self.body_height = rospy.get_param('~body_height', 0.025)

        

        # Create mask for black color        # State        self.step_height = rospy.get_param('~step_height', 0.015)

        mask = cv2.inRange(hsv, self.lower_black, self.upper_black)

                self.line_detected = False        self.x_max = rospy.get_param('~x_max', 0.010)

        # Process ROI

        debug_image = image.copy()        self.line_x = 0        

        line_detected = False

        line_x = 0        self.image_width = self.image_process_size[0]        # For direct joint control in simulation

        

        for i, roi in enumerate(self.line_roi):        self.enabled = True        self.step_size = 0.01  # radians per control cycle

            y_min = int(roi[0] * self.image_process_size[1])

            y_max = int(roi[1] * self.image_process_size[1])                self.current_step_phase = 0.0

            x_min = int(roi[2] * self.image_process_size[0])

            x_max = int(roi[3] * self.image_process_size[0])        # Publishers        

            

            # Draw ROI on debug image        self.debug_image_pub = rospy.Publisher('/line_follower_node/debug_image', Image, queue_size=1)    def calculate_control(self, line_x, image_width):

            cv2.rectangle(debug_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 1)

                    self.line_detected_pub = rospy.Publisher('/line_follower_node/line_detected', Bool, queue_size=1)        """

            # Extract ROI from mask

            roi_mask = mask[y_min:y_max, x_min:x_max]        self.walking_param_pub = rospy.Publisher('/app/set_walking_param', AppWalkingParam, queue_size=1)        Calculate walking control parameters based on line position.

            

            # Find contours in ROI (compatible with OpenCV 3 and 4)        self.walking_command_pub = rospy.Publisher('/app/set_action', String, queue_size=1)        

            contours_result = cv2.findContours(roi_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # OpenCV 4 returns (image, contours, hierarchy), OpenCV 3 returns (contours, hierarchy)                Args:

            contours = contours_result[0] if len(contours_result) == 2 else contours_result[1]

                    # Subscribers            line_x: X coordinate of detected line center

            if contours:

                # Find largest contour        rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=1)            image_width: Width of the image

                largest_contour = max(contours, key=cv2.contourArea)

                area = cv2.contourArea(largest_contour)                    

                

                if area > 50:  # Minimum area threshold        # Wait for ainex_controller to be ready        Returns:

                    # Calculate center of contour

                    M = cv2.moments(largest_contour)        rospy.sleep(2.0)            tuple: (x_output, yaw_output) - forward step and turning angle

                    if M["m00"] != 0:

                        cx = int(M["m10"] / M["m00"]) + x_min                """

                        cy = int(M["m01"] / M["m00"]) + y_min

                                # Enable walking control        center_x = image_width / 2 + self.center_x_offset

                        # Draw contour and center point

                        cv2.drawContours(debug_image, [largest_contour + np.array([x_min, y_min])], -1, (0, 0, 255), 2)        rospy.loginfo("Enabling walking controller...")        

                        cv2.circle(debug_image, (cx, cy), 5, (255, 0, 0), -1)

                                cmd_msg = String()        # Calculate yaw (turning) based on line position

                        line_detected = True

                        line_x = cx        cmd_msg.data = "enable_control"        if abs(line_x - center_x) < 10:

                        break  # Exit if line found

                self.walking_command_pub.publish(cmd_msg)            yaw_output = 0

        # Draw center line

        center_x = int(self.image_process_size[0] / 2)        rospy.sleep(0.5)        elif abs(line_x - center_x) < image_width / 6:

        cv2.line(debug_image, (center_x, 0), (center_x, self.image_process_size[1]), (255, 255, 0), 1)

                            sign = 1 if (line_x - center_x) > 0 else -1

        # Add text

        status_text = "Line Detected: {}".format("YES" if line_detected else "NO")        cmd_msg.data = "start"            yaw_output = sign + self._map_value(

        cv2.putText(debug_image, status_text, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

                self.walking_command_pub.publish(cmd_msg)                line_x - center_x, 

        if line_detected:

            pos_text = "Line X: {}".format(line_x)                        -image_width / 6, 

            cv2.putText(debug_image, pos_text, (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

                rospy.loginfo("Line follower node initialized")                image_width / 6,

        return line_detected, line_x, debug_image

                            self.yaw_range[0] + 1, 

    def control_walking(self):

        """Send walking parameters to ainex_controller based on line detection."""    def image_callback(self, msg):                self.yaw_range[1] - 1

        if not self.enabled:

            return        """Process incoming camera images to detect the line."""            )

        

        walking_param = AppWalkingParam()        try:        else:

        walking_param.speed = self.walking_speed

        walking_param.height = self.body_height            # Convert ROS Image message to OpenCV image (manual conversion)            yaw_output = self.yaw_range[1] if (line_x - center_x) > 0 else self.yaw_range[0]

        

        if self.line_detected:            cv_image = imgmsg_to_cv2(msg)        

            # Calculate error from center

            center_x = self.image_width / 2.0            if cv_image is None:        # Reduce forward speed when turning

            error = (self.line_x - center_x) / center_x  # Normalized -1 to 1

                            return        if abs(yaw_output) > 6:

            # Calculate turn angle

            turn_angle = -error * self.turn_gain * self.max_turn_angle        except Exception as e:            x_output = 0.008

            turn_angle = max(-self.max_turn_angle, min(self.max_turn_angle, turn_angle))

                        rospy.logerr("Image conversion error: {}".format(e))        else:

            # Set walking parameters

            walking_param.x = self.forward_speed  # Move forward            return            x_output = self.x_max

            walking_param.y = 0.0  # No sideways movement

            walking_param.angle = turn_angle  # Turn based on line position                    

            

            rospy.loginfo_throttle(1.0, "Following line - X: {}, Error: {:.2f}, Turn: {:.1f}°".format(        # Resize image for processing        return x_output, -yaw_output

                self.line_x, error, turn_angle))

        else:        resized = cv2.resize(cv_image, tuple(self.image_process_size))    

            # No line detected - stop

            walking_param.x = 0.0            def _map_value(self, value, in_min, in_max, out_min, out_max):

            walking_param.y = 0.0

            walking_param.angle = 0.0        # Detect line        """Map a value from one range to another."""

            

            rospy.logwarn_throttle(2.0, "No line detected - stopping")        line_detected, line_x, debug_image = self.detect_line(resized)        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

        

        # Publish walking parameters        

        self.walking_param_pub.publish(walking_param)

            # Update state

    def run(self):

        """Main loop - just spin since we're using callbacks."""        self.line_detected = line_detectedclass LineFollowerNode:

        rospy.loginfo("Line follower node running...")

        rospy.spin()        if line_detected:    """



            self.line_x = line_x    Main node for line following in Gazebo simulation.

def main():

    try:            self.image_width = self.image_process_size[0]    Processes camera images and controls robot joints.

        node = LineFollowerNode()

        node.run()            """

    except rospy.ROSInterruptException:

        rospy.loginfo("Line follower node terminated.")        # Publish debug image    



        if self.debug_image_pub.get_num_connections() > 0:    def __init__(self):

if __name__ == '__main__':

    main()            try:        rospy.init_node('line_follower_node', anonymous=False)


                debug_msg = cv2_to_imgmsg(debug_image, "bgr8")        rospy.loginfo("Initializing Line Follower Node for Simulation")

                debug_msg.header.stamp = rospy.Time.now()        

                debug_msg.header.frame_id = "camera_link"        # Line detection parameters

                self.debug_image_pub.publish(debug_msg)        self.image_process_size = rospy.get_param('~image_process_size', [160, 120])

            except Exception as e:        self.line_roi = rospy.get_param('~line_roi', [

                rospy.logerr("Debug image publish error: {}".format(e))            [5.0/12.0, 6.0/12.0, 1.0/4.0, 3.0/4.0],

                    [6.0/12.0, 7.0/12.0, 1.0/4.0, 3.0/4.0],

        # Publish detection status            [7.0/12.0, 8.0/12.0, 1.0/4.0, 3.0/4.0]

        self.line_detected_pub.publish(Bool(line_detected))        ])

                

        # Send walking command based on line detection        # Color threshold for black line detection (HSV)

        self.control_walking()        self.lower_black = np.array(rospy.get_param('~lower_black', [0, 0, 0]))

            self.upper_black = np.array(rospy.get_param('~upper_black', [180, 255, 50]))

    def detect_line(self, image):        

        """        # Control state

        Detect black line in the image using HSV color space.        self.enabled = rospy.get_param('~auto_start', True)

                self.controller = LineFollowerController()

        Args:        self.line_detected = False

            image: Input image (BGR format)        self.line_x = 0

                    self.image_width = self.image_process_size[0]

        Returns:        

            tuple: (line_detected, line_x, debug_image)        # Joint publishers for basic walking motion

        """        self.joint_pubs = {}

        # Convert to HSV        self._setup_joint_publishers()

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)        

                # Subscribers

        # Create mask for black color        image_topic = rospy.get_param('~image_topic', '/camera/image_raw')

        mask = cv2.inRange(hsv, self.lower_black, self.upper_black)        self.image_sub = rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=1)

                

        # Process ROI        # Publishers

        debug_image = image.copy()        self.debug_image_pub = rospy.Publisher('~debug_image', Image, queue_size=1)

        line_detected = False        self.line_detected_pub = rospy.Publisher('~line_detected', Bool, queue_size=1)

        line_x = 0        

                # Control rate

        for i, roi in enumerate(self.line_roi):        self.control_rate = rospy.get_param('~control_rate', 10)

            y_min = int(roi[0] * self.image_process_size[1])        self.rate = rospy.Rate(self.control_rate)

            y_max = int(roi[1] * self.image_process_size[1])        

            x_min = int(roi[2] * self.image_process_size[0])        # Initial pose - prepare to walk

            x_max = int(roi[3] * self.image_process_size[0])        self.init_walking_pose()

                    

            # Draw ROI on debug image        rospy.loginfo("Line Follower Node initialized successfully")

            cv2.rectangle(debug_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 1)    

                def _setup_joint_publishers(self):

            # Extract ROI from mask        """Setup publishers for all joint controllers."""

            roi_mask = mask[y_min:y_max, x_min:x_max]        joints = [

                        'l_hip_yaw', 'l_hip_roll', 'l_hip_pitch', 'l_knee', 'l_ank_pitch', 'l_ank_roll',

            # Find contours in ROI (compatible with OpenCV 3 and 4)            'r_hip_yaw', 'r_hip_roll', 'r_hip_pitch', 'r_knee', 'r_ank_pitch', 'r_ank_roll',

            contours_result = cv2.findContours(roi_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)            'l_sho_pitch', 'r_sho_pitch',

            # OpenCV 4 returns (image, contours, hierarchy), OpenCV 3 returns (contours, hierarchy)            'head_pan', 'head_tilt'

            contours = contours_result[0] if len(contours_result) == 2 else contours_result[1]        ]

                    

            if contours:        for joint in joints:

                # Find largest contour            topic = '/{}_controller/command'.format(joint)

                largest_contour = max(contours, key=cv2.contourArea)            self.joint_pubs[joint] = rospy.Publisher(topic, Float64, queue_size=1)

                area = cv2.contourArea(largest_contour)    

                    def init_walking_pose(self):

                if area > 50:  # Minimum area threshold        """Initialize robot to walking ready pose."""

                    # Calculate center of contour        rospy.loginfo("Setting initial walking pose...")

                    M = cv2.moments(largest_contour)        rospy.sleep(0.5)  # Wait for publishers to connect

                    if M["m00"] != 0:        

                        cx = int(M["m10"] / M["m00"]) + x_min        # Set all leg joints to standing position (bent knees for stable standing)

                        cy = int(M["m01"] / M["m00"]) + y_min        init_positions = {

                                    'l_hip_yaw': 0.0,

                        # Draw contour and center point            'l_hip_roll': 0.0,

                        cv2.drawContours(debug_image, [largest_contour + np.array([x_min, y_min])], -1, (0, 0, 255), 2)            'l_hip_pitch': -0.35,  # Lean slightly forward

                        cv2.circle(debug_image, (cx, cy), 5, (255, 0, 0), -1)            'l_knee': 0.7,          # Bend knees for stable standing

                                    'l_ank_pitch': -0.35,   # Ankle angle to keep feet flat

                        line_detected = True            'l_ank_roll': 0.0,

                        line_x = cx            'r_hip_yaw': 0.0,

                        break  # Exit if line found            'r_hip_roll': 0.0,

                    'r_hip_pitch': -0.35,   # Lean slightly forward

        # Draw center line            'r_knee': 0.7,          # Bend knees for stable standing

        center_x = int(self.image_process_size[0] / 2)            'r_ank_pitch': -0.35,   # Ankle angle to keep feet flat

        cv2.line(debug_image, (center_x, 0), (center_x, self.image_process_size[1]), (255, 255, 0), 1)            'r_ank_roll': 0.0,

                    'l_sho_pitch': 0.0,

        # Add text            'r_sho_pitch': 0.0,

        status_text = "Line Detected: {}".format("YES" if line_detected else "NO")            'head_pan': 0.0,

        cv2.putText(debug_image, status_text, (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)            'head_tilt': -0.5,  # Tilt down to look at the ground (negative = down)

                }

        if line_detected:        

            pos_text = "Line X: {}".format(line_x)        for joint, position in init_positions.items():

            cv2.putText(debug_image, pos_text, (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)            if joint in self.joint_pubs:

                        self.joint_pubs[joint].publish(Float64(position))

        return line_detected, line_x, debug_image        

            rospy.sleep(1.0)

    def control_walking(self):        rospy.loginfo("Initial pose set")

        """Send walking parameters to ainex_controller based on line detection."""    

        if not self.enabled:    def image_callback(self, msg):

            return        """Process incoming camera images to detect the line."""

                try:

        walking_param = AppWalkingParam()            # Convert ROS Image message to OpenCV image (manual conversion)

        walking_param.speed = self.walking_speed            cv_image = imgmsg_to_cv2(msg)

        walking_param.height = self.body_height            if cv_image is None:

                        return

        if self.line_detected:        except Exception as e:

            # Calculate error from center            rospy.logerr("Image conversion error: {}".format(e))

            center_x = self.image_width / 2.0            return

            error = (self.line_x - center_x) / center_x  # Normalized -1 to 1        

                    # Resize image for processing

            # Calculate turn angle        resized = cv2.resize(cv_image, tuple(self.image_process_size))

            turn_angle = -error * self.turn_gain * self.max_turn_angle        

            turn_angle = max(-self.max_turn_angle, min(self.max_turn_angle, turn_angle))        # Detect line

                    line_detected, line_x, debug_image = self.detect_line(resized)

            # Set walking parameters        

            walking_param.x = self.forward_speed  # Move forward        # Update state

            walking_param.y = 0.0  # No sideways movement        self.line_detected = line_detected

            walking_param.angle = turn_angle  # Turn based on line position        if line_detected:

                        self.line_x = line_x

            rospy.loginfo_throttle(1.0, "Following line - X: {}, Error: {:.2f}, Turn: {:.1f}°".format(            self.image_width = self.image_process_size[0]

                self.line_x, error, turn_angle))        

        else:        # Publish debug image

            # No line detected - stop        if self.debug_image_pub.get_num_connections() > 0:

            walking_param.x = 0.0            try:

            walking_param.y = 0.0                debug_msg = cv2_to_imgmsg(debug_image, "bgr8")

            walking_param.angle = 0.0                debug_msg.header.stamp = rospy.Time.now()

                            debug_msg.header.frame_id = "camera_link"

            rospy.logwarn_throttle(2.0, "No line detected - stopping")                self.debug_image_pub.publish(debug_msg)

                    except Exception as e:

        # Publish walking parameters                rospy.logerr("Debug image publish error: {}".format(e))

        self.walking_param_pub.publish(walking_param)        

            # Publish detection status

    def run(self):        self.line_detected_pub.publish(Bool(line_detected))

        """Main loop - just spin since we're using callbacks."""    

        rospy.loginfo("Line follower node running...")    def detect_line(self, image):

        rospy.spin()        """

        Detect black line in the image using HSV color space.

        

def main():        Args:

    try:            image: Input image (BGR format)

        node = LineFollowerNode()            

        node.run()        Returns:

    except rospy.ROSInterruptException:            tuple: (line_detected, line_x, debug_image)

        rospy.loginfo("Line follower node terminated.")        """

        # Convert to HSV

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

if __name__ == '__main__':        

    main()        # Create mask for black color

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
            
            # Find contours in ROI (compatible with OpenCV 3 and 4)
            contours_result = cv2.findContours(roi_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # OpenCV 4 returns (image, contours, hierarchy), OpenCV 3 returns (contours, hierarchy)
            contours = contours_result[0] if len(contours_result) == 2 else contours_result[1]
            
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
            
            # Implement basic walking gait
            # Create oscillating motion for walking
            t = rospy.get_time()
            period = self.params['period_time'] / 1000.0  # Convert to seconds
            phase = (t % period) / period  # 0 to 1
            
            # Walking parameters
            step_height = self.params['step_height']
            body_height = self.params['body_height']
            yaw_radians = math.radians(yaw_output * 0.3)  # Scale for turning
            
            # Base standing position (bent knees for stability)
            base_hip_pitch = -0.35
            base_knee = 0.7
            base_ank_pitch = -0.35
            
            # Simple alternating gait (left leg swings when phase < 0.5, right leg when phase >= 0.5)
            if phase < 0.5:
                # Left leg swing phase
                swing_phase = phase * 2  # 0 to 1 during left swing
                
                # Left leg: lift and swing forward from base position
                l_hip_pitch = base_hip_pitch + 0.15 * math.sin(swing_phase * math.pi)
                l_knee = base_knee + 0.3 * math.sin(swing_phase * math.pi)
                l_ank_pitch = base_ank_pitch + 0.15 * math.sin(swing_phase * math.pi)
                
                # Right leg: support (maintain base standing position)
                r_hip_pitch = base_hip_pitch
                r_knee = base_knee
                r_ank_pitch = base_ank_pitch
                
            else:
                # Right leg swing phase
                swing_phase = (phase - 0.5) * 2  # 0 to 1 during right swing
                
                # Right leg: lift and swing forward from base position
                r_hip_pitch = base_hip_pitch + 0.15 * math.sin(swing_phase * math.pi)
                r_knee = base_knee + 0.3 * math.sin(swing_phase * math.pi)
                r_ank_pitch = base_ank_pitch + 0.15 * math.sin(swing_phase * math.pi)
                
                # Left leg: support (maintain base standing position)
                l_hip_pitch = base_hip_pitch
                l_knee = base_knee
                l_ank_pitch = base_ank_pitch
            
            # Apply walking motion
            self.joint_pubs['l_hip_pitch'].publish(Float64(l_hip_pitch))
            self.joint_pubs['l_knee'].publish(Float64(l_knee))
            self.joint_pubs['l_ank_pitch'].publish(Float64(l_ank_pitch))
            
            self.joint_pubs['r_hip_pitch'].publish(Float64(r_hip_pitch))
            self.joint_pubs['r_knee'].publish(Float64(r_knee))
            self.joint_pubs['r_ank_pitch'].publish(Float64(r_ank_pitch))
            
            # Apply yaw for turning
            self.joint_pubs['l_hip_yaw'].publish(Float64(yaw_radians))
            self.joint_pubs['r_hip_yaw'].publish(Float64(-yaw_radians))
            
            # Keep hip roll stable
            self.joint_pubs['l_hip_roll'].publish(Float64(0.0))
            self.joint_pubs['r_hip_roll'].publish(Float64(0.0))
            
            # Keep ankle roll stable
            self.joint_pubs['l_ank_roll'].publish(Float64(0.0))
            self.joint_pubs['r_ank_roll'].publish(Float64(0.0))
            
            rospy.loginfo_throttle(1.0, "Following line - X: {}, Yaw: {:.2f}°, Phase: {:.2f}".format(
                self.line_x, yaw_output, phase))
        else:
            # No line detected - stop walking (return to stable standing stance)
            self.joint_pubs['l_hip_pitch'].publish(Float64(-0.35))
            self.joint_pubs['l_knee'].publish(Float64(0.7))
            self.joint_pubs['l_ank_pitch'].publish(Float64(-0.35))
            
            self.joint_pubs['r_hip_pitch'].publish(Float64(-0.35))
            self.joint_pubs['r_knee'].publish(Float64(0.7))
            self.joint_pubs['r_ank_pitch'].publish(Float64(-0.35))
            
            self.joint_pubs['l_hip_yaw'].publish(Float64(0.0))
            self.joint_pubs['r_hip_yaw'].publish(Float64(0.0))
            
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
