#!/usr/bin/env python3
# encoding: utf-8
"""
Simple camera image publisher for testing line follower without Gazebo camera.
This script can publish test images with lines for development.
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class SimpleCameraPublisher:
    """Publishes test images with a black line for testing."""
    
    def __init__(self):
        rospy.init_node('simple_camera_publisher', anonymous=False)
        
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher('/camera/rgb/image_raw', Image, queue_size=1)
        
        # Image parameters
        self.width = rospy.get_param('~width', 640)
        self.height = rospy.get_param('~height', 480)
        self.rate = rospy.Rate(rospy.get_param('~rate', 30))
        
        # Line parameters
        self.line_width = rospy.get_param('~line_width', 40)
        self.line_position = self.width // 2  # Start at center
        self.line_motion = rospy.get_param('~line_motion', True)
        self.motion_range = rospy.get_param('~motion_range', 200)
        self.motion_speed = 0
        
        rospy.loginfo("Simple Camera Publisher initialized")
        rospy.loginfo("Publishing test images at {}x{} @ {}Hz".format(
            self.width, self.height, rospy.get_param('~rate', 30)))
    
    def generate_test_image(self):
        """Generate a test image with a black line on white background."""
        # Create white background
        image = np.ones((self.height, self.width, 3), dtype=np.uint8) * 255
        
        # Calculate line position (oscillate if motion enabled)
        if self.line_motion:
            self.motion_speed += 0.05
            offset = int(np.sin(self.motion_speed) * self.motion_range)
            self.line_position = self.width // 2 + offset
        
        # Ensure line stays within bounds
        self.line_position = max(self.line_width // 2, 
                                min(self.line_position, self.width - self.line_width // 2))
        
        # Draw black line
        line_start = max(0, self.line_position - self.line_width // 2)
        line_end = min(self.width, self.line_position + self.line_width // 2)
        image[:, line_start:line_end] = [0, 0, 0]  # Black line
        
        # Add some noise for realism
        noise = np.random.randint(0, 20, (self.height, self.width, 3), dtype=np.uint8)
        image = cv2.subtract(image, noise)
        
        return image
    
    def run(self):
        """Main publishing loop."""
        while not rospy.is_shutdown():
            # Generate test image
            image = self.generate_test_image()
            
            # Convert to ROS Image message
            try:
                msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "camera_link"
                self.image_pub.publish(msg)
            except Exception as e:
                rospy.logerr("Error publishing image: {}".format(e))
            
            self.rate.sleep()


def main():
    try:
        publisher = SimpleCameraPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Simple camera publisher terminated.")


if __name__ == '__main__':
    main()
