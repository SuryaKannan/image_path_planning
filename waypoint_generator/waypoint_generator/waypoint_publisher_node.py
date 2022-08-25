import rclpy
from rclpy.node import Node
from planning_interfaces.msg import Waypoint, WaypointArray
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Transform, PoseStamped
from tf2_msgs.msg import TFMessage
import math 
import numpy as np


class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')
        self.waypoints_publisher_ = self.create_publisher(WaypointArray,'global_waypoint',10)
        self.waypoints_visualiser_ = self.create_publisher(MarkerArray,'global_waypoint_marker',0)
        self.tf_subscriber_ = self.create_subscription(TFMessage,  "/tf_static",self.tf_received,10)
        timer_period = 0.5
        self.num_trajectories = 10 # number of trajectories desired
        self.sampling_points = 10 # samples per trajectory
        self.points_array = np.zeros((self.num_trajectories,self.sampling_points,3))
        self.grid_size = (3,2) #(width, height -> y,x using robotics coordinate frame)
        self.timer = self.create_timer(timer_period,self.timer_callback)
        self.waypoints = WaypointArray()
        self.waypoint_markers = MarkerArray()
        self.base_link_height = 0.0
        self.baselnk_2_camera = Transform()
        print("finished creating trajectories!")

    def connect_endpoints(self,point1, point2):
        """
        Connect a set of two cartesian endpoints parabolically using a fixed number of sample.
            point1    -> first endpoint
            point2    -> second endpoint
        """
        m = (point2[1] - point1[1])/(np.cosh(point2[0]) - np.cosh(point1[0]))
        c = point1[1] - m*np.cosh(point1[0])
        x = np.linspace(point1[0], point2[0], self.sampling_points)

        if not math.isinf(m):
            y = m*np.cosh(x) + c
        else:
            y = np.linspace(point1[0], point2[0], self.sampling_points)  
        
        return (x,y)

    def generate_points(self):
        """
        Generate a set of cartesian trajectories based on a given grid size.
        """
        grid_width = (np.ceil(self.grid_size[0]) // 2 ) * 2 + 1 ## round up to nearest odd number 
        y_endpoints = np.linspace(-grid_width, grid_width, self.num_trajectories)
        x_endpoints = self.grid_size[1]*np.ones_like(y_endpoints)

        for trajectory in range(self.num_trajectories):
            y_points,x_points = self.connect_endpoints((y_endpoints[trajectory],x_endpoints[trajectory]),np.array([0,0]))
            z_points = np.zeros_like(y_endpoints)
            points = np.stack((y_points,x_points,z_points), axis=-1)
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
                waypoint.pose.position.x = self.points_array[trajectory,point,1]
                waypoint.pose.position.y = self.points_array[trajectory,point,0]
                waypoint.pose.position.z = self.base_link_height
                waypoint.velocity.linear.x = 0.0
                self.waypoints.waypoints.append(waypoint)

                marker = Marker()
                marker.header.frame_id = waypoint.frame_id
                marker.ns = 'global_waypoint'
                marker.id = id_count
                marker.scale.x = 0.5
                marker.scale.y = 0.5
                marker.scale.z = 0.5
                marker.pose = waypoint.pose
                marker.color.a = 0.5
                marker.color.g = 1.0
                self.waypoint_markers.markers.append(marker)
                id_count+=1
    
    def convert_pose(self,pose,from_frame):
        """
        Convert a pose or transform between frames using tf.
        pose            -> A geometry_msgs.msg/Pose that defines the robots position and orientation in a reference_frame
        from_frame      -> A string that defines the original reference_frame of the robot
        to_frame        -> A string that defines the desired reference_frame of the robot to convert to
        """
        spose = PoseStamped()
        spose.pose = pose
        spose.header.stamp = Node.get_clock(self).now().to_msg()
        spose.header.frame_id = from_frame

        p2 = tf2_geometry_msgs.do_transform_pose(spose, self.baselnk_2_camera)

        return p2.pose
    
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
    
    def set_camera_waypoints(self):
        """
        Convert world waypoints to camera waypoints
        """
        for idx, waypoint in enumerate(self.waypoints.waypoints):
            camera_pose = self.convert_pose(waypoint.pose,"base_link")
            self.waypoints[idx].pose = camera_pose
            self.waypoints[idx].frame_id = "camera_depth_link"

    def timer_callback(self):
        """
        Run steps for waypoint generation and publish 
        """
        self.generate_points()
        self.set_world_waypoints()
        self.set_camera_waypoints()
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