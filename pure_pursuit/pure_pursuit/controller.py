#!/usr/bin/env python3
import numpy as np
import math
import rclpy
from rclpy.node import Node
import ros2_numpy 
from planning_interfaces.msg import WaypointArray, WaypointInfo
from nav_msgs.msg import Odometry
from pure_pursuit.utils import helper
  
class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.pose_subscriber_ = self.create_subscription(Odometry,"/odom",self.odom_received,10)
        self.waypoint_subscriber_ = self.create_subscription(WaypointArray,"/local_waypoints",self.waypoints_received,10)
        self.waypoint_info_subscriber_ = self.create_subscription(WaypointInfo,"/local_waypoint_info",self.waypoints_info_received,10)

        self.local_waypoints = None ## (10,3) local waypoints 
        self.pose = np.array([0,0])        
        self.num_trajectories = 0
        self.num_samples = 0
        self.prev_waypoint_index = None
        self.curr_waypoint_index = 0

        # PP Parameters
        self.k = 0.1  # look forward gain
        self.Lfc = 2.0  # [m] look-ahead distance
        self.Kp = 1.0  # speed proportional gain

    def waypoints_received(self,msg):
        if msg is not None and self.num_samples > 0:
            points_array = np.zeros(shape=(self.num_samples*self.num_trajectories,3))
            waypoints = msg.waypoints

            for index, waypoint in enumerate(waypoints):
                mat = ros2_numpy.numpify(waypoint.pose)
                points_array[index,:] = mat[:-1,-1]

            self.local_waypoints = points_array[7*self.num_samples:8*self.num_samples,:]
            self.waypoint_subscriber_.destroy() ## destroy subscription after storing waypoints
    
    def waypoints_info_received(self,msg):
        if msg is not None:
            self.num_trajectories = msg.trajectories
            self.num_samples = msg.samples
            self.waypoint_info_subscriber_.destroy()

    def odom_received(self,msg):
        self.pose = np.array([msg.pose.pose.position.x,msg.pose.pose.position.y])
        self.closest_waypoint()

    def closest_waypoint(self):
        if self.local_waypoints is not None:
            diffs = self.local_waypoints[:,:2] - self.pose
            dists = np.hypot(diffs[:,0],diffs[:,1])[::-1]
            # [3.98307981 3.62567038 3.26827144 2.91088685 2.55352263 2.19618873 1.83890282 1.48169962 1.12465795 0.7680031 ] example output


def main(args = None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    print("Pure pursuit path tracking simulation start")
    main()