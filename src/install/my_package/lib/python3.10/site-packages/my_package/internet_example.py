import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray

import cv2, math, time


class DroneNode(Node):

    def __init__(self):
        super().__init__('drone_node')
        self.publisher_ = self.create_publisher(Image, 'camera_frame', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        # Create a VideoCapture object
        # The argument '0' gets the default webcam.
        self.cap = cv2.VideoCapture(0)
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
          Int16MultiArray, 
          'drive_topic', 
          self.listener_callback, 
          10)
        self.subscription # prevent unused variable warning

        self.array = Int16MultiArray()

    def listener_callback(self, data):
        """
        Callback function.
        """
        # Display the message on the console
        print("Inside listener callback")
        self.get_logger().info('Receiving drone driving instructions')
        speed = data.data[0]
        fb = data.data[1]
        print(speed, fb)

    def timer_callback(self):
        # Capture frame-by-frame
        # This method returns True/False as well
        # as the video frame.
        ret, img = self.cap.read()
        img = cv2.resize(img, (360, 240))
        image_message = self.br.cv2_to_imgmsg(img)
        # cv2.imshow("Image", img)
        # cv2.waitKey(1)
        if ret == True:
            self.publisher_.publish(image_message)
            self.get_logger().info('Publishing images')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    drone_node = DroneNode()

    rclpy.spin(drone_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    drone_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
