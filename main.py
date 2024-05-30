#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header
import cv2
import numpy as np
from collections import Counter


class DetermineColor(Node):
    def __init__(self):
        super().__init__('color_detector')
        self.image_sub = self.create_subscription(Image, '/camera/color/image_raw', self.callback, 10)
        self.color_pub = self.create_publisher(Header, '/rotate_cmd', 10)
        self.bridge = CvBridge()

    def callback(self, data):
        try:
            # Listen to the image topic
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            # Prepare rotate_cmd msg
            # DO NOT DELETE THE BELOW THREE LINES!
            msg = Header()
            msg.stamp = data.header.stamp
            msg.frame_id = '0'  # default: STOP

            # Determine background color
            dominant = self.color_detector(image)
            if dominant is not None:
                if dominant == 'B':
                    msg.frame_id = '+1'  # CCW
                elif dominant == 'R':
                    msg.frame_id = '-1'  # CW
                elif dominant == 'else':
                    msg.frame_id = '0'  # STOP

            # Publish color_state
            self.color_pub.publish(msg)
        except CvBridgeError as e:
            self.get_logger().error(f'Failed to convert image: {e}')

    def color_detector(self, image):
        #TODO
        bgr_img = image
        hsv_img = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2HSV)
        image = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2GRAY)

        ret, thresh = cv2.threshold(image, 20, 255, cv2.THRESH_BINARY)
        edges = cv2.Canny(thresh, 200, 250)
        contours,_ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        largest_area = 0
        best_contour = None
        roi = None
        epsilon_n = 0.03
        found = False
        skipLength = False
	
        while not found:
        	for contour in contours:
            		epsilon = epsilon_n*cv2.arcLength(contour,True)
            		approx = cv2.approxPolyDP(contour,epsilon,True)
            		area = cv2.contourArea(approx)
            		if (len(approx) == 4 or skipLength) and cv.isContourConvex(approx):
            			if (area > 2000 and area > largest_area):
            				found = True
            				best_contour = approx
            				largest_area = area
            	
        	epsilon += 0.01
        	if (epsilon > 0.2):
            		skipLength = True
            		epsilon = 0.03
            				
        mask = np.zeros(hsv_img.shape[:2], dtype=np.uint8)
        cv2.fillPoly(mask, [best_contour], 1)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2,2))
        mask = cv2.morphologyEx(mask, cv2.MORPH_ERODE, kernel)
        roi = cv2.bitwise_and(hsv_img, hsv_img, mask=mask)
        
        
        R1 = cv2.inRange(roi, (0,50,50), (10,255,255))
        R2 = cv2.inRange(roi, (170,50,50), (180,255,255))
        R = cv2.bitwise_or(R1,R2)
        G = cv2.inRange(roi, (50,50,50), (70, 255, 255))
        B = cv2.inRange(roi, (110,50,50), (130, 255, 255))

        color_counts = {'R': cv2.countNonZero(R),
                      'B': cv2.countNonZero(B),
                      'G': cv2.countNonZero(G),
                      'else': 0}
	
        dominant = max(color_counts, key=color_counts.get)
        if color_counts['R'] < 2000 and color_counts['B'] < 2000:
        	dominant = 'else'
	

        print(dominant)
        print(color_counts[dominant])
        return dominant

if __name__ == '__main__':
    rclpy.init()
    detector = DetermineColor()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()
