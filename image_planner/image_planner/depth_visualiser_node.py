#!/usr/bin/env python3
from re import S
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
import ros2_numpy 
import image_geometry
from planning_interfaces.msg import WaypointArray, WaypointInfo
from tf2_msgs.msg import TFMessage
from image_planner.utils import transforms
        
class DepthVisualiser(Node):
    def __init__(self):
        super().__init__('depth_visualiser')
        self.depth_subscriber_ = self.create_subscription(Image,"/camera/depth/image_rect_raw",self.depth_received,10)
        self.image_subscriber_ = self.create_subscription(Image,"/camera/color/image_raw",self.image_received,10)
        self.tf_subscriber_ = self.create_subscription(TFMessage,"/tf_static",self.tf_received,10)
        self.camera_info_subscriber_ = self.create_subscription(CameraInfo,"/camera/depth/camera_info",self.camera_info_received,10)
        self.waypoint_subscriber_ = self.create_subscription(WaypointArray,"/local_waypoints",self.waypoints_received,10)
        self.goal_subscriber_ = self.create_subscription(PoseStamped,"/goal_pose",self.goal_received,10)
        self.waypoint_info_subscriber_ = self.create_subscription(WaypointInfo,"/local_waypoint_info",self.waypoints_info_received,10)
        self.pose_subscriber_ = self.create_subscription(Odometry,"/odom",self.odom_received,10)
        self.image_publisher_ = self.create_publisher(Image, '/waypoint_overlay', 10)
        # timer_period = 1/10
        # self.timer = self.create_timer(timer_period,self.timer_callback)
        self.points_array = None
        self.camera_traj_pixels = None
        self.z_truth = None
        self.global_waypoint_pixel = np.array([[0],[0],[0],[1]])
        self.pose = np.array([0,0])
        self.camera_model = None 
        self.baselnk_2_camera = None
        self.IM_HEIGHT = 480
        self.IM_WIDTH = 640
        self.num_trajectories = 0
        self.num_samples = 0

    # def timer_callback(self):
    #     if self.points_array is not None and :

    def odom_received(self,msg):
        self.pose = np.array([msg.pose.pose.position.x,msg.pose.pose.position.y])
    
    def goal_received(self,msg):
        if msg is not None and self.baselnk_2_camera is not None and self.camera_model is not None:
            print(self.pose)
            global_waypoint = np.array([[msg.pose.position.x-self.pose[0]],[msg.pose.position.y-self.pose[1]],[0],[1]])
            global_waypoint_pixel = transforms.transform_points(self.baselnk_2_camera, global_waypoint)
            self.global_waypoint_pixel = transforms.project_points(self.camera_model.intrinsicMatrix(),global_waypoint_pixel)
            self.global_waypoint_pixel[0] = np.clip(self.global_waypoint_pixel[0], a_min = 0, a_max = self.IM_WIDTH)
            self.global_waypoint_pixel[1] = np.clip(self.global_waypoint_pixel[1], a_min = 0, a_max = self.IM_HEIGHT)

    def image_received(self,msg):
        current_frame = ros2_numpy.numpify(msg)
        if self.camera_traj_pixels is not None:
            current_frame[self.camera_traj_pixels[1,:],self.camera_traj_pixels[0,:],:] = np.array([255,0,0])
            current_frame[self.global_waypoint_pixel[1],self.global_waypoint_pixel[0],:] = np.array([0,255,0])
            img = ros2_numpy.msgify(Image, current_frame,encoding="rgb8")
            self.image_publisher_.publish(img)

    def depth_received(self,msg):
        depth_frame = ros2_numpy.numpify(msg)
        depth_frame = np.array(depth_frame, dtype=np.float32)
        self.tentacle_select(depth_frame)
        # center_idx = np.array(depth_frame.shape) // 2
        # print ('center depth:', depth_frame[center_idx[0], center_idx[1]])

    def tentacle_select(self,measured_depth):
        if self.camera_traj_pixels is not None:
            z_measure = measured_depth[self.camera_traj_pixels[1,:],self.camera_traj_pixels[0,:]]
            print([z_measure.shape])
            # diff = z_measure-self.z_truth
            # print(diff)

    
    def waypoints_received(self,msg):
        if msg is not None and self.num_samples > 0 and self.baselnk_2_camera is not None and self.camera_model is not None:
            self.points_array = np.zeros(shape=(self.num_samples*self.num_trajectories,3))
            waypoints = msg.waypoints

            for index, waypoint in enumerate(waypoints):
                mat = ros2_numpy.numpify(waypoint.pose)
                self.points_array[index,:] = mat[:-1,-1]
            
            self.points_array= self.points_array.T
            homogen = np.ones(self.points_array.shape[1])
            self.points_array = np.vstack([self.points_array,homogen])
            camera_points = transforms.transform_points(self.baselnk_2_camera, self.points_array)
            self.camera_traj_pixels = transforms.project_points(self.camera_model.intrinsicMatrix(),camera_points)
            self.z_truth = self.points_array[0,:]
            self.waypoint_subscriber_.destroy() ## destroy subscription after storing waypoints
    
    def waypoints_info_received(self,msg):
        if msg is not None:
            self.num_trajectories = msg.trajectories
            self.num_samples = msg.samples
            self.waypoint_info_subscriber_.destroy()
    
    def camera_info_received(self,msg):
        ## http://docs.ros.org/en/kinetic/api/image_geometry/html/python/index.html#module-image_geometry
        if msg is not None:
            self.camera_model = image_geometry.PinholeCameraModel()
            self.camera_model.fromCameraInfo(msg)
            self.camera_info_subscriber_.destroy() ## destroy subscription after storing camera params
    
    def tf_received(self,msg):
        """
        Collect relevant tf information to build base_link to depth_camera transformation
        """
        if msg is not None:
            baselnk_2_camera = msg.transforms[2].transform
            baselnk_2_camera.rotation = msg.transforms[1].transform.rotation ## combine rotation and translation to create one tf between base_link and depth_camera
            self.baselnk_2_camera = ros2_numpy.numpify(baselnk_2_camera)
            self.tf_subscriber_.destroy() ## destroy subscription after storing transformations

def main(args=None):
    rclpy.init(args=args)
    depth_visualiser = DepthVisualiser()
    rclpy.spin(depth_visualiser)
    depth_visualiser.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()