#!/usr/bin/python3 
import rclpy
from rclpy.node import Node
from planning_interfaces.msg import Waypoint, WaypointArray
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Transform
from tf2_msgs.msg import TFMessage
import math 
import numpy as np
import image_geometry
import time
from waypoint_generator.utils import storage

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')
        self.waypoints_publisher_ = self.create_publisher(WaypointArray,'global_waypoints',10)
        self.waypoints_visualiser_ = self.create_publisher(MarkerArray,'global_waypoint_markers',0)
        self.tf_subscriber_ = self.create_subscription(TFMessage,  "/tf_static",self.tf_received,10)
        self.camera_info_subscriber_ = self.create_subscription(CameraInfo,"/camera/depth/camera_info",self.info_received,10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period,self.timer_callback)
        self.camera_model = None 
        self.IM_HEIGHT = 480 ## see Gazebo camera definition TODO: add args parser  https://github.com/oKermorgant/simple_launch
        self.IM_WIDTH = 640 
        self.path = "../../params"
        self.num_trajectories = 15 # number of trajectories desired TODO: add args parser 
        self.sampling_points = 10 # samples per trajectory
        self.points_array = np.zeros((self.num_trajectories,self.sampling_points,3))
        self.grid_width = 0.85 ## TODO: add args parser 
        self.grid_size = (0,0)
        self.waypoints = WaypointArray()
        self.waypoint_markers = MarkerArray()
        self.base_link_height = 0.0
        self.baselnk_2_camera = Transform()
        self.published = False

    def connect_endpoints(self,point1, point2):
        """
        Connect a set of two cartesian endpoints parabolically using a fixed number of sample.
            point1 -> first endpoint
            point2 -> second endpoint
        """
        m = (point2[1] - point1[1])/(np.cosh(point2[0]) - np.cosh(point1[0]))
        c = point1[1] - m*np.cosh(point1[0])
        y = np.linspace(point1[0], point2[0], self.sampling_points)

        if not math.isinf(m):  ## check if line has no gradient (i.e oriented with x axis)
            x = m*np.cosh(y) + c
        else:
            x = np.linspace(point1[1], point2[0], self.sampling_points)  
        
        return (y,x)

    def generate_points(self):
        """
        Generate a set of cartesian trajectories based on a given grid size.
        """
        y_endpoints = np.linspace(-self.grid_size[0], self.grid_size[0], self.num_trajectories)
        x_endpoints = self.grid_size[1]*np.ones_like(y_endpoints)

        for trajectory in range(self.num_trajectories):
            y_points,x_points = self.connect_endpoints((y_endpoints[trajectory],x_endpoints[trajectory]),np.array([0,0]))
            z_points = np.zeros(self.sampling_points)
            points = np.stack((x_points,y_points,z_points), axis=-1)
            points = np.nan_to_num(points)
            self.points_array[trajectory,:,:] = points

    def set_world_waypoints(self):
        """
        Publish 3D world points based on a fixed world frame
        """
        id_count = 0
        for trajectory in range(self.num_trajectories):
            for point in range(self.sampling_points):
                waypoint = Waypoint()
                waypoint.frame_id = 'base_link' ## this is the origin of the robot frame
                waypoint.pose.position.x = self.points_array[trajectory,point,0]
                waypoint.pose.position.y = self.points_array[trajectory,point,1]
                waypoint.pose.position.z = self.base_link_height
                waypoint.pose.orientation.x = 0.0
                waypoint.pose.orientation.y = 0.0
                waypoint.pose.orientation.z = 0.0
                waypoint.pose.orientation.w = 1.0
                waypoint.velocity.linear.x = 0.0
                self.waypoints.waypoints.append(waypoint)

                marker = Marker()
                marker.header.frame_id = waypoint.frame_id
                marker.type = 2
                marker.ns = 'global_waypoint'
                marker.id = id_count
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.pose = waypoint.pose
                marker.color.a = 0.5
                marker.color.r = 1.0
                self.waypoint_markers.markers.append(marker)
                id_count+=1
    
    def tf_received(self,msg):
        """
        Collect relevant tf information to build base_link to depth_camera transformation
        """
        if msg is not None:
            self.base_link_height = msg.transforms[0].transform.translation.z ## set world trajectories at correct height
            baselnk_2_camera = msg.transforms[2].transform
            baselnk_2_camera.rotation = msg.transforms[1].transform.rotation ## combine rotation and translation to create one tf between base_link and depth_camera
            self.baselnk_2_camera = baselnk_2_camera
            self.tf_subscriber_.destroy() ## destroy subscription after storing transformations

    def info_received(self,msg):
        """
        Collect and store camera intrinsics
        http://docs.ros.org/en/kinetic/api/image_geometry/html/python/index.html#module-image_geometry
        """
        camera_model = image_geometry.PinholeCameraModel()
        camera_model.fromCameraInfo(msg)
        storage.update_param(self.path,"intrinsic.npy",camera_model.intrinsicMatrix())
        self.grid_size = (self.grid_width,(self.grid_width/self.IM_WIDTH)*camera_model.fx()) #(width, height -> y,x using robotics coordinate frame)
        self.camera_info_subscriber_.destroy() ## destroy subscription after storing camera params
    
    def timer_callback(self):
        """
        Run steps for waypoint generation and publish
        """
        if not self.published:
            time.sleep(1) ## wait until camera params are stored
            self.generate_points()
            time.sleep(1) ## wait until waypoints have been generated fully 
            print(self.points_array)
            self.set_world_waypoints()
            storage.update_param(self.path,"waypoints.npy",self.points_array)
            self.waypoints_publisher_.publish(self.waypoints)
            self.waypoints_visualiser_.publish(self.waypoint_markers)
            self.published = True
            print("finished creating trajectories!")
        else:
            self.waypoints_publisher_.publish(self.waypoints)
            self.waypoints_visualiser_.publish(self.waypoint_markers)
    
def main(args=None):
    rclpy.init(args=args)
    waypoint_publisher = WaypointPublisher()
    rclpy.spin(waypoint_publisher)
    waypoint_publisher.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()