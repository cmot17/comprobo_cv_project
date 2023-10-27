import rclpy
from threading import Thread
from rclpy.node import Node
import time
from sensor_msgs.msg import Image
from copy import deepcopy
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist, Vector3
from collections import deque
from imutils.video import VideoStream
import argparse


class NeatoFetch(Node):
    """ """

    def __init__(self, image_topic):
        """ """
        super().__init__("neato_fetch")
        self.cv_image = None  # the latest image from the camera
        self.bridge = CvBridge()  # used to convert ROS messages to OpenCV
        self.create_subscription(Image, image_topic, self.process_image, 10)
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        thread = Thread(target=self.loop_wrapper)
        thread.start()

    def process_image(self, msg):
        """Process image messages from ROS and stash them in an attribute
        called cv_image for subsequent processing"""

        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)

    def loop_wrapper(self):
        cv2.namedWindow("video_window")
        cv2.namedWindow("binary_window")

        self.hue_lower_bound = 27
        self.saturation_lower_bound = 137
        self.value_lower_bound = 97
        self.hue_upper_bound = 40  # Hue ranges from 0-180 in OpenCV
        self.saturation_upper_bound = 255
        self.value_upper_bound = 255

        cv2.createTrackbar("Hue Lower", "binary_window", self.hue_lower_bound, 180, self.set_hue_lower_bound)
        cv2.createTrackbar("Saturation Lower", "binary_window", self.saturation_lower_bound, 255, self.set_saturation_lower_bound)
        cv2.createTrackbar("Value Lower", "binary_window", self.value_lower_bound, 255, self.set_value_lower_bound)
        cv2.createTrackbar("Hue Upper", "binary_window", self.hue_upper_bound, 180, self.set_hue_upper_bound)
        cv2.createTrackbar("Saturation Upper", "binary_window", self.saturation_upper_bound, 255, self.set_saturation_upper_bound)
        cv2.createTrackbar("Value Upper", "binary_window", self.value_upper_bound, 255, self.set_value_upper_bound)

        cv2.setMouseCallback("video_window", self.process_mouse_event)
        while True:
            self.run_loop()
            time.sleep(0.1)

    def set_hue_lower_bound(self, val):
        self.hue_lower_bound = val

    def set_saturation_lower_bound(self, val):
        self.saturation_lower_bound = val

    def set_value_lower_bound(self, val):
        self.value_lower_bound = val

    def set_hue_upper_bound(self, val):
        self.hue_upper_bound = val

    def set_saturation_upper_bound(self, val):
        self.saturation_upper_bound = val

    def set_value_upper_bound(self, val):
        self.value_upper_bound = val

    def process_mouse_event(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # Update this method to show the color info directly on the main window
            b, g, r = self.cv_image[y, x]
            hsv = cv2.cvtColor(self.cv_image[y:y+1, x:x+1], cv2.COLOR_BGR2HSV)[0][0]
            info_text = "BGR: (%d, %d, %d), HSV: (%d, %d, %d)" % (b, g, r, hsv[0], hsv[1], hsv[2])
            cv2.putText(self.cv_image, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    def run_loop(self):
        if self.cv_image is not None:
            self.hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
            self.binary_image = cv2.inRange(
                self.hsv_image,
                (self.hue_lower_bound, self.saturation_lower_bound, self.value_lower_bound),
                (self.hue_upper_bound, self.saturation_upper_bound, self.value_upper_bound)
            )
            self.binary_image = cv2.erode(self.binary_image, None, iterations=2)
            self.binary_image = cv2.dilate(self.binary_image, None, iterations=2)

             # Find contours
            contours, _ = cv2.findContours(self.binary_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                perimeter = cv2.arcLength(contour, True)
                area = cv2.contourArea(contour)
                
                # Avoid division by zero and small contours
                # cv2.drawContours(self.cv_image, [contour], 0, (0,255,0), 3)
                print(f'Contour detected with perimeter {perimeter} and area {area}')
                if perimeter > 0 and area > 50:
                    circularity = 4 * np.pi * area / (perimeter * perimeter)
                    print(f'Perimeter and area met threshold, circularity: {circularity}')
                    if 0.25 <= circularity <= 1.2:  # Adjust thresholds as needed
                        (x, y), radius = cv2.minEnclosingCircle(contour)
                        center = (int(x), int(y))
                        radius = int(radius)
                        cv2.circle(self.cv_image, center, radius, (0, 255, 0), 2)
            cv2.imshow("video_window", self.cv_image)
            cv2.imshow("binary_window", self.binary_image)
            cv2.waitKey(5)


if __name__ == "__main__":
    node = NeatoFetch("/camera/image_raw")
    node.run()


def main(args=None):
    rclpy.init()
    n = NeatoFetch("camera/image_raw")
    rclpy.spin(n)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
