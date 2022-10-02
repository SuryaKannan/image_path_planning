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
from planning_interfaces.msg import Tentacle
from geometry_msgs.msg import PoseStamped
        
class TentaclePlanner(Node):
    def __init__(self):
        super().__init__('tentacle_planner')
        self.depth_subscriber_ = self.create_subscription(Image,"/camera/depth/image_rect_raw",self.depth_received,10)
        self.image_subscriber_ = self.create_subscription(Image,"/camera/color/image_raw",self.image_received,10)
        self.tf_subscriber_ = self.create_subscription(TFMessage,"/tf_static",self.tf_received,10)
        self.camera_info_subscriber_ = self.create_subscription(CameraInfo,"/camera/depth/camera_info",self.camera_info_received,10)
        self.waypoint_subscriber_ = self.create_subscription(WaypointArray,"/local_waypoints",self.waypoints_received,10)
        self.waypoint_info_subscriber_ = self.create_subscription(WaypointInfo,"/local_waypoint_info",self.waypoints_info_received,10)
        self.goal_subscriber_ = self.create_subscription(PoseStamped,"/goal_pose",self.goal_received,10)
        self.image_publisher_ = self.create_publisher(Image, '/waypoint_overlay', 10)
        self.tentacle_publisher_ = self.create_publisher(Tentacle, '/tentacle_selection', 10)
     
        self.camera_traj_pixels = None
        self.points_array = None
        self.z_truth = None
        self.camera_model = None 
        self.baselnk_2_camera = None
        self.visualise = True
        self.goal = None
        self.traj_collisions = None
        self.IM_HEIGHT = 480
        self.IM_WIDTH = 640
        self.num_trajectories = 0
        self.num_samples = 0
        self.crit_dist = 0.3 ## depth comparision threshold. Lower values (>0.5) should be chosen for images containing less noise 
        self.tentacle = Tentacle()

    def tentacle_select(self,measured_depth):
        '''
        Short term horizon, image based planner. To summarise the steps involved:

        1) Compare measured depth to desired depth according to 3D waypoints that are pre-made  
        2) Calculate the number of potential collisions using a threshold for each trajectory
        3) Perform cost calculation for each trajectory using Euclidian distance from waypoints and goal pose
        4) Find all trajectories that satisfy a safe collision threshold (i.e no collisions or less than a given number)
        5) Choose trajectory with lowest cost that also has the least amount of collisions
        '''
        if self.camera_traj_pixels is not None and self.z_truth is not None:
            z_measure = measured_depth[self.camera_traj_pixels[1,:],self.camera_traj_pixels[0,:]]
            diff = 0.5*self.z_truth - z_measure 
            diff = np.reshape(diff,(self.num_trajectories,self.num_samples))
            diff[diff > self.crit_dist] = 1
            diff[diff <= self.crit_dist] = 0
            traj_collisions = diff.sum(axis=1) ## number of collisions per trajectory
            # traj_collisions = traj_collisions[::-1] ## uncomment for tentacle readability

            traj_costs = self.points_array[0:2,:]
            traj_costs = np.linalg.norm(self.goal[:, None]-traj_costs,axis=0)
            traj_costs = np.reshape(traj_costs,(self.num_trajectories,self.num_samples))
            traj_costs = np.sum(traj_costs,axis=1) ## costs of trajectories relative to goal
            # traj_costs = traj_costs[::-1] ## uncomment for tentacle readability

            if traj_costs[traj_collisions == 0].shape != 0:
                #Running into ValueError: zero-size array to reduction operation minimum which has no identity
                safe_trajs = np.min(traj_costs[traj_collisions == 0]) ## safest trajectory with lowest cost
                optimal_traj = np.where(traj_costs == safe_trajs) ## optimal trajectory index
                self.tentacle.tentacle_id = int(optimal_traj[0])
                self.tentacle_publisher_.publish(self.tentacle)
                
    def goal_received(self,msg):
        '''
        Storing goal pose 
        '''
        if msg is not None:
            self.goal = np.array([msg.pose.position.x,msg.pose.position.y])
            
    def depth_received(self,msg):
        '''
        Following https://www.ros.org/reps/rep-0118.html to read depth.

        For every frame, the planner is called 
        '''
        depth_frame = ros2_numpy.numpify(msg)
        depth_frame = np.array(depth_frame, dtype=np.float32)
        if self.goal is not None:
            self.tentacle_select(depth_frame)

    def image_received(self,msg):
        """
        If Visualisation is on, an image is published to the /waypoint_overlay topic where the projected waypoints can be seen.
        """
        if self.camera_traj_pixels is not None and self.visualise is True:
            current_frame = ros2_numpy.numpify(msg)
            current_frame[self.camera_traj_pixels[1,:],self.camera_traj_pixels[0,:],:] = np.array([255,0,0])
            img = ros2_numpy.msgify(Image, current_frame,encoding="rgb8")
            self.image_publisher_.publish(img)

    def waypoints_received(self,msg):
        """
        Receives 3D world waypoints and calculates projected camera coordinates. Also stores true depth to be later used by the planner.

        Two transformations occur:
        1) World points from base_link frame are transformed to the camera_depth_link frame 
        2) Transformed points are then projected back into image space
        """
        if msg is not None and self.num_samples > 0 and self.baselnk_2_camera is not None and self.camera_model is not None:
            self.points_array = np.zeros(shape=(self.num_samples*self.num_trajectories,3))
            waypoints = msg.waypoints

            for index, waypoint in enumerate(waypoints):
                mat = ros2_numpy.numpify(waypoint.pose)
                self.points_array[index,:] = mat[:-1,-1]
            
            self.points_array= self.points_array.T
            homogen = np.ones(self.points_array.shape[1])
            self.points_array = np.vstack([self.points_array,homogen])
            camera_points_array = transforms.transform_points(self.baselnk_2_camera, self.points_array)
            self.camera_traj_pixels = transforms.project_points(self.camera_model.intrinsicMatrix(),camera_points_array)
            self.z_truth = camera_points_array[-2,:]
            self.waypoint_subscriber_.destroy() ## destroy subscription after storing waypoints
    
    def waypoints_info_received(self,msg):
        """
        Store waypoint information
        """
        if msg is not None:
            self.num_trajectories = msg.trajectories
            self.num_samples = msg.samples
            self.waypoint_info_subscriber_.destroy()
    
    def camera_info_received(self,msg):
        """
        http://docs.ros.org/en/kinetic/api/image_geometry/html/python/index.html#module-image_geometry
        
        Store camera intrinsics
        """
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
    tentacle_planner = TentaclePlanner()
    rclpy.spin(tentacle_planner)
    tentacle_planner.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()