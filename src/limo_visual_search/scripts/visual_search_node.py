#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Bool
class VisualSearchNode:
    def __init__(self):
        rospy.init_node("visual_search_node")


        # Load target object image
        target_img_path = rospy.get_param("~target_image")
        self.target_img = cv2.imread(target_img_path, 0)
        if self.target_img is None:
            rospy.logerr("Could not load target image!")
            exit()

        # ORB detector
        self.orb = cv2.ORB_create(1000)
        self.kp_target, self.des_target = self.orb.detectAndCompute(self.target_img, None)

        # Camera subscriber
        self.bridge = CvBridge()
        rospy.Subscriber("/limo/color/image_raw", Image, self.camera_callback)

        # Publisher: True/False
        self.result_pub = rospy.Publisher("/visual_search/target_found", 
                                          Bool, queue_size=1)

        rospy.loginfo("Visual search node running...")
        rospy.spin()

    def camera_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Extract ORB features
        kp_frame, des_frame = self.orb.detectAndCompute(gray, None)
        if des_frame is None:
            return

        # Match descriptors
        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        matches = bf.match(self.des_target, des_frame)
        matches = sorted(matches, key=lambda x: x.distance)

        # Determine if object is found
        GOOD_MATCH_THRESHOLD = 25  # tune this value
        if len(matches) > GOOD_MATCH_THRESHOLD:
            rospy.loginfo("TARGET OBJECT FOUND!")
            self.result_pub.publish(True)
            
        else:
            self.result_pub.publish(False)
            

if __name__ == "__main__":
    VisualSearchNode()
