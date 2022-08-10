import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
from cv_bridge import CvBridge

class DepthVisualiser(Node):
    def __init__(self):
        super().__init__('depth_visualiser')
        self.depth_subscriber_ = self.create_subscription(
            CompressedImage,  
            "/camera/depth/image_rect_raw/compressed",
            self.depth_received,
            10
        )
        self.publisher_ = self.create_publisher(CompressedImage, '/image_planner_vis', 10)
        self.bridge = CvBridge()
        
    def depth_received(self,msg):
        img = self.bridge.compressed_imgmsg_to_cv2(msg)
        print(img)

def main(args=None):
    rclpy.init(args=args)
    depth_visualiser = DepthVisualiser()
    rclpy.spin(depth_visualiser)
    depth_visualiser.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()