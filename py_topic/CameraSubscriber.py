import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

class RGBDSubscriber(Node):

    def __init__(self, topic_name, filename):
        super().__init__('py_RGBD_sub_node')
        self.subscriber_ = self.create_subscription(Image, topic_name, self.subscribe_message, 10)
        self.subscriber_  # prevent unused variable warning
        
        self.vidInit = False
        self.vid = None
        self.filename = filename

    def subscribe_message(self, msg):
        # self.get_logger().info('Recieved - Linear Velocity : %f, Angular Velocity : %f' % (msg.linear.x, msg.angular.z))

        # convert to numpy array
        image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
	
        if not self.vidInit:
            h, w = image.shape[:2]
            fourcc = cv.VideoWriter_fourcc('m', 'p', '4', 'v')
            self.vid = cv2.VideoWriter(self.filename, fourcc, 25, (h,w))
        
        # image = cv2.resize(image, (self.h, self.w))
        self.vid.write(image)
        
    def write_video(self):
        if self.vid:
            self.vid.release()
        

def main(args=None):
    if len(args)<2:
        print('python CameraSubscriber.py topic_name save_filename args')
        exit(0)
        
    topic_name = args[0]
    filename = args[1]
    
    rclpy.init(args=args[2:])
    minimal_subscriber = RGBDSubscriber(topic_name, filename)
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.write_video()
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
