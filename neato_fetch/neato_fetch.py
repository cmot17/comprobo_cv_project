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
        self.most_circular_contour = None

        self.state = "go_to_ball"



    def process_image(self, msg):
        """Process image messages from ROS and stash them in an attribute
        called cv_image for subsequent processing"""

        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.hsv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        
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


    def process_mouse_event(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # Update this method to show the color info directly on the main window
            b, g, r = self.cv_image[y, x]
            hsv = cv2.cvtColor(self.cv_image[y:y+1, x:x+1], cv2.COLOR_BGR2HSV)[0][0]
            info_text = "BGR: (%d, %d, %d), HSV: (%d, %d, %d)" % (b, g, r, hsv[0], hsv[1], hsv[2])
            cv2.putText(self.cv_image, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    def go_to_ball(self):
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

        highest_circularity = 0
        
        for contour in contours:
            perimeter = cv2.arcLength(contour, True)
            area = cv2.contourArea(contour)

            if perimeter > 0 and area > 50:  # Avoid division by zero and small contours
                circularity = 4 * np.pi * area / (perimeter * perimeter)
                if 0.25 <= circularity <= 1.2 and circularity > highest_circularity:  # Adjust thresholds as needed
                    highest_circularity = circularity
                    self.most_circular_contour = contour
            print(f'Most circular contour has perimeter: {perimeter}, area: {area}, and circularity: {highest_circularity}')

        height, width = self.cv_image.shape[:2]
        print(f"Width: {width}, Height: {height}")
                
        fwd_vel = 0.0
        rot_vel = 0.0
        # Draw the most circular contour
        if self.most_circular_contour is not None:
            (x, y), radius = cv2.minEnclosingCircle(self.most_circular_contour)
            center = (int(x), int(y))
            radius = int(radius)
            cv2.circle(self.cv_image, center, radius, (0, 255, 0), 2)
                
            if self.most_circular_contour is not None:
                if self.possession() == True:
                    rot_vel = 0.0
                    fwd_vel = 0.0
                    # self.state = "locate_marker"
                    # COMMENTED RIGHT NOW ONLY BECAUSE THE LOCATE MARKER STATE HAS NOT BEEN DEFINED
                    # MEANING ONCE IT SWTICHES TO LOCATE_MARKER IT NEVER SWITCHES BACK TO GO_TO_BALL, NOT ALLOWING US TO TEST THINGS
                    print(f'BALL IS IN POSSESSION BABYYYY but at the moment we are just stopped with it tho.')

                else:
                    rot_vel = -1 * (x - (width / 2)) / (width / 2)
                    fwd_vel = 0.2
                    print(f'fwd vel = {fwd_vel}, rot vel = {rot_vel}')
    
            
        print("messsage published")
        self.pub.publish(Twist(linear=Vector3(x=fwd_vel,y=0.0,z=0.0), angular=Vector3(x=0.0,y=0.0,z=rot_vel)))
    def possession(self):
            height, width = self.cv_image.shape[:2]
            print(f"Width: {width}, Height: {height}")
                
            start_point = (width -100,height)
            end_point = (100,300)
            self.possession_rect = cv2.rectangle(self.cv_image, start_point, end_point,(255, 0, 0), 2)

            x_ref = 100
            y_ref = 300
            width_ref = width
            height_ref = height
            self.ref_box = (x_ref, y_ref, width_ref, height_ref)

    # Iterate over contours and check
            for contour in self.most_circular_contour:
                # Get the bounding box of the current contour
                x, y, w, h = cv2.boundingRect(contour)

                # Check if the contour's bounding box is within the reference bounding box
                if (x >= x_ref and y >= y_ref and x + w <= x_ref + width_ref and y + h <= y_ref + height_ref):
                    print("Contour's bounding box is within the reference bounding box.")
                    return True
                else:
                    print("Contour's bounding box is not within the reference bounding box.")
                    return False
                
    def detect_and_go_to_aruco(self, frame):
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        aruco_params = cv2.aruco.DetectorParameters()
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray_frame, aruco_dict, parameters=aruco_params)
        if ids is not None:
            if 4 in ids:
                target_index = np.squeeze(np.where(ids == 4))
                target_corners = corners[target_index]

                center = np.mean(target_corners[0], axis=0)
                center = tuple(center.astype(int))

                x_error = center[0] - frame.shape[1] // 2
                angular_vel = -0.001 * x_error
                fwd_vel = 0.2  

                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                cv2.circle(frame, center, 5, (0, 255, 0), -1)

                self.pub.publish(Twist(
                    linear=Vector3(x=fwd_vel, y=0.0, z=0.0),
                    angular=Vector3(x=0.0, y=0.0, z=angular_vel)
                ))
        else:
            self.pub.publish(Twist(
                linear=Vector3(x=0.0, y=0.0, z=0.0),
                angular=Vector3(x=0.0, y=0.0, z=0.0)
            ))

    def origin_search(self):
        pass

    
        
    def run_loop(self):
        #if self.cv_image is not None: 
          #  if (self.state == "go_to_ball"):
              #  self.go_to_ball()

        if self.cv_image is not None:
            self.detect_and_go_to_aruco(self.cv_image)
            
            cv2.imshow("video_window", self.cv_image)
            cv2.imshow("binary_window", self.binary_image)
            cv2.waitKey(5)
def main(args=None):
    rclpy.init()
    node = NeatoFetch("camera/image_raw")
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
