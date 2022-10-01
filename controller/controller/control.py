#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
import ros2_numpy 
from planning_interfaces.msg import WaypointArray, WaypointInfo
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
  
class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.pose_subscriber_ = self.create_subscription(Odometry,"/odom",self.odom_received,10)
        self.waypoint_subscriber_ = self.create_subscription(WaypointArray,"/local_waypoints",self.waypoints_received,10)
        self.waypoint_info_subscriber_ = self.create_subscription(WaypointInfo,"/local_waypoint_info",self.waypoints_info_received,10)
        self.goal_subscriber_ = self.create_subscription(PoseStamped,"/goal_pose",self.goal_received,10)
        self.velocity_publisher_ = self.create_publisher(Twist,'/cmd_vel',10)
        timer_period = 1/10 ## 10Hz 
        self.timer = self.create_timer(timer_period,self.timer_callback)

        self.local_waypoints = None 
        self.pose = np.array([0,0])   
        self.goal = np.array([0,0])      
        self.num_trajectories = 0
        self.num_samples = 0
        self.goal_range = 0.1 
        self.max_speed = 1
        self.slow_dist = 0.5
        self.motion = Twist()

    def waypoints_received(self,msg):
        if msg is not None and self.num_samples > 0:
            points_array = np.zeros(shape=(self.num_samples*self.num_trajectories,3))
            waypoints = msg.waypoints

            for index, waypoint in enumerate(waypoints):
                mat = ros2_numpy.numpify(waypoint.pose)
                points_array[index,:] = mat[:-1,-1]

            self.local_waypoints = points_array
            self.waypoint_subscriber_.destroy() ## destroy subscription after storing waypoints
    
    def waypoints_info_received(self,msg):
        if msg is not None:
            self.num_trajectories = msg.trajectories
            self.num_samples = msg.samples
            self.waypoint_info_subscriber_.destroy() ## destroy subscription after storing waypoint characteristics

    def odom_received(self,msg):
        self.pose = np.array([msg.pose.pose.position.x,msg.pose.pose.position.y])

    def goal_received(self,msg):
        self.goal = np.array([msg.pose.position.x,msg.pose.position.y])

    def calc_vel(self,dist):
        if dist > self.slow_dist:
            return self.max_speed
        else:
            return (self.max_speed/self.slow_dist)*dist
        
    def timer_callback(self):
        dist_to_goal = np.linalg.norm(self.goal - self.pose)

        if dist_to_goal > self.goal_range:
            self.motion.linear.x = np.float(self.calc_vel(dist_to_goal))
        else: 
            self.motion.linear.x = 0.0
            self.motion.angular.z = 0.0
        
        self.velocity_publisher_.publish(self.motion)

def main(args = None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    print("Pure pursuit path tracking simulation start")
    main()