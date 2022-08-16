#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
import numpy as np
import ros2_numpy 
import cv2
from cv_bridge import CvBridge
import image_geometry

class DepthVisualiser(Node):
    def __init__(self):
        super().__init__('depth_visualiser')
        self.depth_subscriber_ = self.create_subscription(
            PointCloud2,  
            "/camera/depth/color/points",
            self.depth_received,
            10
        )
        self.image_subscriber_ = self.create_subscription(
            Image,  
            "/camera/depth/image_rect_raw",
            self.image_received,
            10
        )
        self.camera_info_subscriber_ = self.create_subscription(
            CameraInfo,  
            "/camera/depth/camera_info",
            self.info_received,
            10
        )
        self.br = CvBridge()
        self.camera_model = None 
        self.IM_HEIGHT = 480
        self.IM_WIDTH = 640
        
    def depth_received(self,msg):
        depth_data = ros2_numpy.numpify(msg)
        depth_frame = np.reshape(depth_data,(self.IM_HEIGHT,self.IM_WIDTH))
        print(self.camera_model.intrinsicMatrix())
    
    def image_received(self,msg):
        current_frame = self.br.imgmsg_to_cv2(msg)
    
    def info_received(self,msg):
        ## http://docs.ros.org/en/kinetic/api/image_geometry/html/python/index.html#module-image_geometry
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(msg)
        self.camera_info_subscriber_.destroy() ## destroy subscription after storing camera params

def main(args=None):
    rclpy.init(args=args)
    depth_visualiser = DepthVisualiser()
    rclpy.spin(depth_visualiser)
    depth_visualiser.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()