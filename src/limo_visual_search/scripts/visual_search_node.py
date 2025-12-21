#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Bool
import cv2
from ultralytics import YOLO
from collections import deque

class AdvancedVisualSearchYOLO:
    def __init__(self):
        rospy.init_node("advanced_visual_search_yolo")

        self.target_image_path = rospy.get_param("~target_image_path", "")
        self.bridge = CvBridge()
        self.history = deque(maxlen=5)

        # Load YOLO model
        self.model = YOLO("yolov8s-oiv7.pt")  # You can change to yolov8s.pt or your custom model


        # Print all class labels the model was trained on
        print("YOLO trained labels:")
        for i, label in self.model.names.items():
            print(f"{i}: {label}")

        # Detect object in target image and get label
        target_img = cv2.imread(self.target_image_path)
        target_img = cv2.resize(target_img, (640, 480))
        target_results = self.model(target_img)
        if len(target_results[0].boxes) == 0:
            rospy.logerr("No objects detected in target image!")
            self.target_label = None
        else:
            # Take first detected object (can be enhanced to pick largest/confident one)
            self.target_label = target_results[0].names[int(target_results[0].boxes.cls[0])]
            rospy.loginfo(f"Target label: {self.target_label}")
        
        # ROS subscribers and publishers
        rospy.Subscriber("/limo/color/image_raw", Image, self.callback, queue_size=1)
        self.pub = rospy.Publisher("/visual_search/target_found", Bool, queue_size=1)

        rospy.spin()

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        frame = cv2.resize(frame, (640, 480))

        results = self.model(frame)
        detected = False

        for box in results[0].boxes:
            label = results[0].names[int(box.cls)]
            if label == self.target_label:
                detected = True
                break

        self.history.append(detected)
        confirmed = all(self.history) and len(self.history) == 5
        self.pub.publish(confirmed)

        rospy.loginfo(f"Detected={detected}, confirmed={confirmed}")



if __name__ == "__main__":
    try:
        AdvancedVisualSearchYOLO()
    except rospy.ROSInterruptException:
        pass
