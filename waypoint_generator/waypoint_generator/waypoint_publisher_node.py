import rclpy
from rclpy.node import Node
from planning_interfaces.msg import Waypoint
from visualization_msgs.msg import Marker

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')
        self.waypoint_publisher_ = self.create_publisher(Waypoint,'global_waypoint',10)
        self.waypoint_visualiser_ = self.create_publisher(Marker,'global_waypoint_marker',0)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period,self.timer_callback)
        self.waypoint = None
        self.waypoint_marker = None
        self.set_waypoint()


    def set_waypoint(self):
        waypoint = Waypoint()
        waypoint.frame_id = 'map'
        waypoint.pose.position.x = 5.0
        waypoint.pose.position.y = -5.0
        waypoint.velocity.linear.x = 0.0
        self.waypoint=waypoint
        marker = Marker()
        marker.header.frame_id = waypoint.frame_id
        marker.ns = 'global_waypoint'
        marker.id = 1
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.pose = waypoint.pose
        marker.color.a = 0.5
        marker.color.g = 1.0
        self.waypoint_marker = marker


    def timer_callback(self):
        self.waypoint_publisher_.publish(self.waypoint)
        self.waypoint_visualiser_.publish(self.waypoint_marker)
    
def main(args=None):
    rclpy.init(args=args)
    waypoint_publisher = WaypointPublisher()
    rclpy.spin(waypoint_publisher)
    waypoint_publisher.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
