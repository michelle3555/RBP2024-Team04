
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header

class DetermineColor(Node):
    def __init__(self):
        super().__init__('color_detector')
        self.image_sub = self.create_subscription(Image, '/color', self.callback, 10)
        self.color_pub = self.create_publisher(Header, '/rotate_cmd', 10)
        self.bridge = CvBridge()
        self.color_r = 0
        self.color_g = 0
        self.color_b = 0
    def callback(self, data):
        try:
            # listen image topic
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            # prepare rotate_cmd msg
            # DO NOT DELETE THE BELOW THREE LINES!
            msg = Header()
            msg = data.header
            msg.frame_id = '1'  # default: STOP
            cv2.imshow('Image',image)
            cv2.waitKey(1)
            self.color_r =  image[1,1,2]

            self.color_g = image[1,1,1]

            self.color_b = image[1,1,0]

            if self.color_r == 255:
            	msg.frame_id = '-1'
            elif self.color_g == 255:
            	msg.frame_id = '0'
            elif self.color_b == 255: 
            	msg.frame_id = '+1'
            self.color_pub.publish(msg)
        except CvBridgeError as e:
            self.get_logger().error('Failed to convert image: %s' % e)


if __name__ == '__main__':
    rclpy.init()
    detector = DetermineColor()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()


