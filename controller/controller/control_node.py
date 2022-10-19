#!/usr/bin/env python3
import numpy as np
import ros2_numpy 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from planning_interfaces.msg import WaypointArray, WaypointInfo, Tentacle

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.pose_subscriber_ = self.create_subscription(Odometry,"/odom",self.odom_received,10)
        self.waypoint_subscriber_ = self.create_subscription(WaypointArray,"/local_waypoints",self.waypoints_received,10)
        self.waypoint_info_subscriber_ = self.create_subscription(WaypointInfo,"/local_waypoint_info",self.waypoints_info_received,10)
        self.goal_subscriber_ = self.create_subscription(PoseStamped,"/goal_pose",self.goal_received,10)
        self.tentacle_subscriber_ = self.create_subscription(Tentacle,"/tentacle_selection",self.tentacle_received,10)
        self.velocity_publisher_ = self.create_publisher(Twist,'/cmd_vel',10)
        timer_period = 1/10 ## 10Hz 
        self.timer = self.create_timer(timer_period,self.timer_callback)

        self.y_dists = None 
        self.y_arc = 1 ## waypoint from all trajectories to create a radius from
        self.pose = np.array([0,0])   
        self.goal = np.array([0,0])      
        self.num_trajectories = 0
        self.num_samples = 0
        self.goal_range = 3
        self.max_speed = 0.5
        self.slow_dist = 2.5
        self.max_angular = 0.7
        self.min_angular = 0.1
        self.goal_set = False
        self.motion = Twist()
        self.tentacle = None
        self.angle_arr = []

    def waypoints_received(self,msg):
        '''
        Receive all waypoints but only store desired waypoints based on radius chosen to create angular velocity.
        This esentially means sampling the same waypoint from each tentacle. 
        '''
        if msg is not None and self.num_samples > 0:
            points_array = np.zeros(shape=(self.num_samples*self.num_trajectories,3))
            waypoints = msg.waypoints

            for index, waypoint in enumerate(waypoints):
                mat = ros2_numpy.numpify(waypoint.pose)
                points_array[index,:] = mat[:-1,-1]

            self.y_dists = np.reshape(points_array[:,1],(self.num_trajectories,self.num_samples))
            self.y_dists = self.y_dists[:,self.y_arc]
            self.get_logger().info(f'{self.y_dists}')
            self.waypoint_subscriber_.destroy() ## destroy subscription after storing waypoints
    
    def waypoints_info_received(self,msg):
        if msg is not None:
            self.num_trajectories = msg.trajectories
            self.num_samples = msg.samples
            self.waypoint_info_subscriber_.destroy() ## destroy subscription after storing waypoint characteristics

    def odom_received(self,msg):
        self.pose = np.array([msg.pose.pose.position.x,msg.pose.pose.position.y])

    def goal_received(self,msg):
        if msg is not None:
            self.goal = np.array([msg.pose.position.x,msg.pose.position.y])
            self.goal_set = True

    def calc_vel(self,dist):
        if dist > self.slow_dist:
            return self.max_speed
        else:
            return (self.max_speed/self.slow_dist**2)*(dist**2) ## follow a parabolic velocity curve for braking 
    
    def calc_angular(self):
        angular = self.y_dists[self.tentacle]
        
        if abs(angular) < 0.1:
            angular = 0
        else:
            angular = (self.max_angular/abs(self.y_dists[0]))*angular + np.sign(angular)*self.min_angular
        
        self.angle_arr.insert(0, angular)

        if len(self.angle_arr) >= 10:
            self.angle_arr.pop(-1)

        return np.average(self.angle_arr)

    def tentacle_received(self,msg):
        self.tentacle = msg.tentacle_id
        
    def timer_callback(self):
        if self.goal_set == True:
            dist_to_goal = np.linalg.norm(self.goal - self.pose)
            if (dist_to_goal > self.goal_range) and self.tentacle is not None:
                vel = np.float(self.calc_vel(dist_to_goal))
                self.motion.linear.x = vel
                self.motion.angular.z = float(self.calc_angular())
                self.get_logger().info(f'w: {self.motion.angular.z}, v: {self.motion.linear.x}')
            else: 
                self.motion.linear.x = 0.0
                self.motion.angular.z = 0.0
                self.goal_set = False

            self.velocity_publisher_.publish(self.motion)

def main(args = None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()