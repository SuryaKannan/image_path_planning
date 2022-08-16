import rclpy
from rclpy.node import Node
from planning_interfaces.msg import Waypoint, WaypointArray
from visualization_msgs.msg import Marker, MarkerArray
import math 
import numpy as np

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')
        self.waypoints_publisher_ = self.create_publisher(WaypointArray,'global_waypoint',10)
        self.waypoints_visualiser_ = self.create_publisher(MarkerArray,'global_waypoint_marker',0)
        timer_period = 0.5
        self.num_trajectories = 5 # number of trajectories desired
        self.sampling_points = 5 # samples per trajectory
        self.points_array = np.zeros((self.num_trajectories,self.sampling_points,3))
        self.grid_size = (3,2) #(width, height -> y,x using robotics coordinate frame)
        self.timer = self.create_timer(timer_period,self.timer_callback)
        self.waypoints = WaypointArray()
        self.waypoint_markers = MarkerArray()
        self.generate_points()
        self.set_waypoints()
        print("finished creating trajectories!")

    def connect_endpoints(self,point1, point2):
        m = (point2[1] - point1[1])/(np.cosh(point2[0]) - np.cosh(point1[0]))
        c = point1[1] - m*np.cosh(point1[0])
        x = np.linspace(point1[0], point2[0], self.sampling_points)

        if not math.isinf(m):
            y = m*np.cosh(x) + c
        else:
            y = np.linspace(point1[0], point2[0], self.sampling_points)  
        
        return (x,y)

    def generate_points(self):
        grid_width = (np.ceil(self.grid_size[0]) // 2 ) * 2 + 1 ## round up to nearest odd number 
        y_endpoints = np.linspace(-grid_width, grid_width, self.num_trajectories)
        x_endpoints = self.grid_size[1]*np.ones_like(y_endpoints)

        for trajectory in range(self.num_trajectories):
            y_points,x_points = self.connect_endpoints((y_endpoints[trajectory],x_endpoints[trajectory]),np.array([0,0]))
            z_points = np.zeros_like(y_endpoints)
            points = np.stack((y_points,x_points,z_points), axis=-1)
            points = np.nan_to_num(points)
            self.points_array[trajectory,:,:] = points

        print(self.points_array)

    def set_waypoints(self):
        id_count = 0
        for trajectory in range(self.num_trajectories):
            for point in range(self.sampling_points):
                waypoint = Waypoint()
                waypoint.frame_id = 'map'
                waypoint.pose.position.x = self.points_array[trajectory,point,0]
                waypoint.pose.position.y = self.points_array[trajectory,point,1]
                waypoint.pose.position.z = self.points_array[trajectory,point,2]
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


    def timer_callback(self):
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
