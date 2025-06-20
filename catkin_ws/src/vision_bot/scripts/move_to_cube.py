#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

class CubeFollower:
    def __init__(self):
        rospy.init_node('cube_follower')
        self.bridge = CvBridge()
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.move_cmd = Twist()
        rospy.spin()

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define red color range in HSV (adjust if needed)
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)

        M = cv2.moments(mask)
        h, w = frame.shape[:2]

        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            # Determine direction
            if cx < w/3:
                # Object is on the left → turn left
                self.move_cmd.linear.x = 0.0
                self.move_cmd.angular.z = 0.3
            elif cx > 2*w/3:
                # Object is on the right → turn right
                self.move_cmd.linear.x = 0.0
                self.move_cmd.angular.z = -0.3
            else:
                # Object is centered → move forward
                self.move_cmd.linear.x = 0.2
                self.move_cmd.angular.z = 0.0
        else:
            # Object not found → stop
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(self.move_cmd)

if __name__ == '__main__':
    CubeFollower()
