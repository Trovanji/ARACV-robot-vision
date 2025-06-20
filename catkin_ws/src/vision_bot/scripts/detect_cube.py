#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CubeDetector:
    def __init__(self):
        rospy.init_node('cube_detector')
        self.bridge = CvBridge()
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        rospy.spin()

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Detect red color (adjust HSV range if your cube is a different color)
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)

        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            width = frame.shape[1]
            if cx < width/3:
                position = "Left"
            elif cx > 2*width/3:
                position = "Right"
            else:
                position = "Center"
            rospy.loginfo("Cube position: " + position)
        else:
            rospy.loginfo("Cube not found")

if __name__ == '__main__':
    CubeDetector()
