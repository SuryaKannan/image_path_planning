#!/usr/bin/env python3
import numpy as np
import ros2_numpy 
import rclpy
from rclpy.node import Node
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import Image, CameraInfo
from tf2_msgs.msg import TFMessage
from image_planner.utils import transforms
from geometry_msgs.msg import PoseStamped
from planning_interfaces.msg import WaypointArray, WaypointInfo, Tentacle

class TentaclePlanner(Node):
    def __init__(self):
        super().__init__('tentacle_planner')
        self.depth_subscriber_ = self.create_subscription(Image,"/camera/depth/image_rect_raw",self.depth_received,10)
        self.image_subscriber_ = self.create_subscription(Image,"/camera/color/image_raw",self.image_received,10)
        self.tf_static_subscriber_ = self.create_subscription(TFMessage,"/tf_static",self.tf_static_received,10)
        self.tf_subscriber_ = self.create_subscription(TFMessage,"/tf",self.tf_received,1000)
        self.camera_info_subscriber_ = self.create_subscription(CameraInfo,"/camera/depth/camera_info",self.camera_info_received,10)
        self.waypoint_subscriber_ = self.create_subscription(WaypointArray,"/local_waypoints",self.waypoints_received,10)
        self.waypoint_info_subscriber_ = self.create_subscription(WaypointInfo,"/local_waypoint_info",self.waypoints_info_received,10)
        self.goal_subscriber_ = self.create_subscription(PoseStamped,"/goal_pose",self.goal_received,10)
        self.image_publisher_ = self.create_publisher(Image, '/waypoint_overlay', 10)
        self.tentacle_publisher_ = self.create_publisher(Tentacle, '/tentacle_selection', 10)
        
        self.visualise = False
        self.plan_to_goal = False
        self.IM_HEIGHT = 480
        self.IM_WIDTH = 640
        self.num_trajectories = 0
        self.num_samples = 0
        self.crit_dist = 0.3 ## depth comparision threshold. Lower values (>0.5) should be chosen for images containing less noise 
        self.relative_goal = np.array([0,0,0,1])

        self.goal = None
        self.camera_traj_pixels = None
        self.points_array = None
        self.traj_collisions = None
        self.z_truth = None
        self.baselnk_2_camera = None
        self.basefootprint_2_odom = None
        
        self.camera_model = PinholeCameraModel()
        self.tentacle = Tentacle()

    def tentacle_select(self,measured_depth):
        '''
        Short term horizon, image based planner. To summarise the steps involved:

        1) Calculate relative goal position between odometry and base_link  
        1) Compare measured depth to desired depth according to 3D waypoints that are pre-made  
        2) Calculate the number of potential collisions using a threshold for each trajectory
        3) Perform cost calculation for each trajectory using Euclidian distance from updated waypoints and goal pose
        4) Find all trajectories that satisfy a safe collision threshold (i.e no collisions or less than a given number)
        5) Choose trajectory with lowest cost that also has the least amount of collisions
        '''
        if self.goal is not None and self.basefootprint_2_odom is not None and self.camera_traj_pixels is not None:
            self.relative_goal[0] = self.goal[0] - self.basefootprint_2_odom[0,3]
            self.relative_goal[1] = self.goal[1] - self.basefootprint_2_odom[1,3]
            base_link_goal_pos = transforms.transform_points(self.basefootprint_2_odom, self.relative_goal)
            
            z_measure = measured_depth[self.camera_traj_pixels[1,:],self.camera_traj_pixels[0,:]]
            diff = 0.5*self.z_truth - z_measure 
            diff[diff > self.crit_dist] = 1
            diff[diff <= self.crit_dist] = 0
            diff = np.reshape(diff,(self.num_trajectories,self.num_samples))
            traj_collisions = diff.sum(axis=1) ## number of collisions per trajectory
            # self.get_logger().info(f'{traj_collisions[::-1]}') ## uncomment to see collisions per path 
            
            traj_costs = self.points_array[0:2,:]
            traj_costs = np.linalg.norm(base_link_goal_pos[0:2, None]-traj_costs,axis=0)
            traj_costs = np.reshape(traj_costs,(self.num_trajectories,self.num_samples))
            traj_costs = np.sum(traj_costs,axis=1) ## costs of trajectories relative to goal
            # self.get_logger().info(f'{traj_costs[::-1]}')  ## uncomment to see tentacle costs 

            if traj_costs[traj_collisions == 0].shape != 0:
                safe_trajs = np.min(traj_costs[traj_collisions == 0]) ## safest trajectory with lowest cost
                optimal_traj = np.where(traj_costs == safe_trajs) ## optimal trajectory index
                self.tentacle.tentacle_id = int(optimal_traj[0])
                self.tentacle_publisher_.publish(self.tentacle)
            
    def depth_received(self,msg):
        '''
        Following https://www.ros.org/reps/rep-0118.html to read depth.

        Planner is called for every new frame received
        '''
        depth_frame = ros2_numpy.numpify(msg)
        depth_frame = np.array(depth_frame, dtype=np.float32)
        self.tentacle_select(depth_frame)
            
    def image_received(self,msg):
        """
        If Visualisation is set to True, an image is published to the /waypoint_overlay topic where the projected waypoints can be seen.
        """
        if self.visualise is True and self.camera_traj_pixels is not None:
            current_frame = ros2_numpy.numpify(msg)
            current_frame[self.camera_traj_pixels[1,:],self.camera_traj_pixels[0,:],:] = np.array([255,0,0]) ## white projected waypoint pixels
            img = ros2_numpy.msgify(Image, current_frame,encoding="rgb8")
            self.image_publisher_.publish(img)

    def waypoints_received(self,msg):
        """
        Receives 3D world waypoints and calculates projected camera coordinates. Also stores true depth to be later used by the planner.

        Two transformations occur:
        1) World points from base_link frame are transformed to the camera_depth_link frame 
        2) Transformed points are then projected back into image space
        """
        if self.num_samples > 0 and self.baselnk_2_camera is not None:
            self.points_array = np.zeros(shape=(self.num_samples*self.num_trajectories,3))
            waypoints = msg.waypoints

            for index, waypoint in enumerate(waypoints):
                mat = ros2_numpy.numpify(waypoint.pose)
                self.points_array[index,:] = mat[:-1,-1]
            
            ## create homogenous world points
            self.points_array= self.points_array.T
            homogen = np.ones(self.points_array.shape[1])
            self.points_array = np.vstack([self.points_array,homogen])

            ## perform 3D projection into image space 
            camera_points_array = transforms.transform_points(self.baselnk_2_camera, self.points_array)
            self.camera_traj_pixels = transforms.project_points(self.camera_model.intrinsicMatrix(),camera_points_array)
            self.z_truth = camera_points_array[-2,:]

            self.waypoint_subscriber_.destroy() ## destroy subscription after storing waypoints
    
    def goal_received(self,msg):
        '''
        Storing goal pose as homogenous coordinate and then transforming it to base_footprint.
        We need to initially account for the relative position of the goal coordinates to odom
        '''
        if self.basefootprint_2_odom is not None: 
            self.goal = np.array([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z,1])
            
    def waypoints_info_received(self,msg):
        """
        Store waypoint information
        """
        self.num_trajectories = msg.trajectories
        self.num_samples = msg.samples
    
    def camera_info_received(self,msg):
        """
        http://docs.ros.org/en/kinetic/api/image_geometry/html/python/index.html#module-image_geometry
        
        Store camera intrinsics
        """
        self.camera_model.fromCameraInfo(msg)
    
    def tf_received(self,msg):
        """
        Collect relevant tf information to build base_footprint to odom transformation
        """
        transforms = msg.transforms[0]
        if transforms.header.frame_id == "odom":
            self.basefootprint_2_odom = ros2_numpy.numpify(transforms.transform)

    def tf_static_received(self,msg):
        """
        Collect relevant tf information to build base_link to depth_camera transformation
        """
        baselnk_2_camera = msg.transforms[2].transform
        baselnk_2_camera.rotation = msg.transforms[1].transform.rotation ## combine rotation and translation to create one tf between base_link and depth_camera
        self.baselnk_2_camera = ros2_numpy.numpify(baselnk_2_camera)
        self.tf_static_subscriber_.destroy() ## destroy subscription after storing transformations

def main(args=None):
    rclpy.init(args=args)
    tentacle_planner = TentaclePlanner()
    rclpy.spin(tentacle_planner)
    tentacle_planner.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
