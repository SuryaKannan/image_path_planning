#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
import ros2_numpy 
import image_geometry
from planning_interfaces.msg import WaypointArray, WaypointInfo
from tf2_msgs.msg import TFMessage
from image_planner.utils import transforms
from planning_interfaces.msg import TentacleArray
        
class DepthVisualiser(Node):
    def __init__(self):
        super().__init__('depth_visualiser')
        self.depth_subscriber_ = self.create_subscription(Image,"/camera/depth/image_rect_raw",self.depth_received,10)
        self.image_subscriber_ = self.create_subscription(Image,"/camera/color/image_raw",self.image_received,10)
        self.tf_subscriber_ = self.create_subscription(TFMessage,"/tf_static",self.tf_received,10)
        self.camera_info_subscriber_ = self.create_subscription(CameraInfo,"/camera/depth/camera_info",self.camera_info_received,10)
        self.waypoint_subscriber_ = self.create_subscription(WaypointArray,"/local_waypoints",self.waypoints_received,10)
        self.waypoint_info_subscriber_ = self.create_subscription(WaypointInfo,"/local_waypoint_info",self.waypoints_info_received,10)
        self.image_publisher_ = self.create_publisher(Image, '/waypoint_overlay', 10)
        self.tentacle_publisher_ = self.create_publisher(TentacleArray, '/Tentacle_selection', 10)
        # timer_period = 1/10
        # self.timer = self.create_timer(timer_period,self.timer_callback)
        self.camera_traj_pixels = None
        self.z_truth = None
        self.camera_model = None 
        self.baselnk_2_camera = None
        self.visualise = True
        self.IM_HEIGHT = 480
        self.IM_WIDTH = 640
        self.num_trajectories = 0
        self.num_samples = 0
        self.crit_dist = 0.4

    # def timer_callback(self):
    #     if self.points_array is not None and :

    def tentacle_select(self,measured_depth):
        if self.camera_traj_pixels is not None and self.z_truth is not None:
            z_measure = measured_depth[self.camera_traj_pixels[1,:],self.camera_traj_pixels[0,:]]
            diff = 0.5*self.z_truth - z_measure
            diff = np.reshape(diff,(self.num_trajectories,self.num_samples))
            diff[diff > self.crit_dist] = 1
            diff[diff <= self.crit_dist] = 0
            diff_traj = diff.sum(axis=1)
            diff_traj = diff_traj[::-1]
            
            
    def depth_received(self,msg):
        '''
        Following https://www.ros.org/reps/rep-0118.html
        '''
        depth_frame = ros2_numpy.numpify(msg)
        depth_frame = np.array(depth_frame, dtype=np.float32)
        self.tentacle_select(depth_frame)

    def image_received(self,msg):
        if self.camera_traj_pixels is not None and self.visualise is True:
            current_frame = ros2_numpy.numpify(msg)
            current_frame[self.camera_traj_pixels[1,:],self.camera_traj_pixels[0,:],:] = np.array([255,0,0])
            img = ros2_numpy.msgify(Image, current_frame,encoding="rgb8")
            self.image_publisher_.publish(img)

    def waypoints_received(self,msg):
        if msg is not None and self.num_samples > 0 and self.baselnk_2_camera is not None and self.camera_model is not None:
            points_array = np.zeros(shape=(self.num_samples*self.num_trajectories,3))
            waypoints = msg.waypoints

            for index, waypoint in enumerate(waypoints):
                mat = ros2_numpy.numpify(waypoint.pose)
                points_array[index,:] = mat[:-1,-1]
            
            points_array= points_array.T
            homogen = np.ones(points_array.shape[1])
            points_array = np.vstack([points_array,homogen])
            camera_points_array = transforms.transform_points(self.baselnk_2_camera, points_array)
            self.camera_traj_pixels = transforms.project_points(self.camera_model.intrinsicMatrix(),camera_points_array)
            self.z_truth = camera_points_array[-2,:]
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