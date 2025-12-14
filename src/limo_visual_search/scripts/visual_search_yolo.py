#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool
from ultralytics import YOLO


class VisualSearchYOLO:
    def __init__(self):
        rospy.init_node("visual_search_yolo")

        # Parameters
        self.target_class = rospy.get_param("~target_class", "bottle")
        model_path = rospy.get_param("~model_path", "yolov8n.pt")
        self.conf_threshold = rospy.get_param("~conf_threshold", 0.4)
        self.img_size = rospy.get_param("~img_size", 640)

        # Load YOLO model
        try:
            self.model = YOLO(model_path)
            rospy.loginfo(f"Loaded YOLO model from {model_path}")
        except Exception as e:
            rospy.logerr(f"Failed to load YOLO model '{model_path}': {e}")
            rospy.signal_shutdown("Model load failed")
            return

        self.bridge = CvBridge()

        # ROS subscribers and publishers
        rospy.Subscriber("/limo/color/image_raw", Image, self.image_callback)
        self.result_pub = rospy.Publisher("/visual_search/target_found", Bool, queue_size=1)

        rospy.loginfo("YOLO visual search node started...")
        rospy.spin()

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logwarn(f"Failed to convert image: {e}")
            return

        if frame is None or frame.size == 0:
            rospy.logwarn("Received empty frame!")
            return

        # Ensure 3 channels
        if len(frame.shape) == 2:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        elif frame.shape[2] != 3:
            rospy.logwarn(f"Unexpected number of channels: {frame.shape[2]}")
            return

        # Resize to square input
        frame_resized = cv2.resize(frame, (self.img_size, self.img_size), interpolation=cv2.INTER_LINEAR)
        frame_resized = np.ascontiguousarray(frame_resized, dtype=np.uint8)

        # YOLO inference
        try:
            results = self.model.predict(
                frame_resized.copy(),
                conf=self.conf_threshold,
                imgsz=self.img_size,
                verbose=False,
                max_det=10,
            )
        except Exception as e:
            rospy.logwarn(f"YOLO inference failed: {e}")
            return

        # Check for target class with confidence filtering
        found = False
        for r in results:
            boxes = getattr(r, 'boxes', None)
            if boxes is None or len(boxes) == 0:
                continue

            for box in boxes:
                cls_id = int(box.cls[0])
                conf = float(box.conf[0])
                class_name = self.model.names.get(cls_id, "")

                if class_name == self.target_class and conf >= self.conf_threshold:
                    found = True
                    break
            if found:
                break

        # Publish result
        self.result_pub.publish(found)


if __name__ == "__main__":
    try:
        VisualSearchYOLO()
    except rospy.ROSInterruptException:
        pass
