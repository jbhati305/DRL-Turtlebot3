# drl_navigation/drl_navigation/controller.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformListener, Buffer
from math import atan2, sqrt, pi
import tf_transformations

class Controller(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.get_logger().info('Controller Node Started')

        # Subscribers
        self.path_sub = self.create_subscription(Path, '/global_path', self.path_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path = []
        self.obstacle_detected = False

        # Control loop timer
        self.timer = self.create_timer(0.1, self.control_loop) # 10 Hz

    def path_callback(self, msg: Path):
        self.path = msg.poses
        self.get_logger().info('Received new path.')

    def scan_callback(self, msg: LaserScan):
        # Simple obstacle detection in front of the robot
        # Check a 30-degree cone in front
        num_ranges = len(msg.ranges)
        front_ranges = msg.ranges[:15] + msg.ranges[-15:]
        min_dist = min(r for r in front_ranges if r > 0.0) # Ignore 0 readings
        
        self.obstacle_detected = min_dist < 0.35 # Obstacle if something is closer than 35 cm

    def control_loop(self):
        twist_msg = Twist()

        if self.obstacle_detected:
            self.get_logger().warn('Obstacle detected! Stopping.')
            self.cmd_vel_pub.publish(twist_msg) # Publish zero Twist to stop
            return

        if not self.path:
            return

        # Get robot's current pose from TF
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except Exception as e:
            self.get_logger().error(f'Could not get transform: {e}')
            return

        # Get current position and orientation (yaw)
        current_x = transform.transform.translation.x
        current_y = transform.transform.translation.y
        q = transform.transform.rotation
        _, _, current_yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

        # Get the next waypoint from the path
        # For simplicity, we just aim for the final goal.
        # A better controller would follow each waypoint sequentially.
        goal_pose = self.path[-1]
        goal_x = goal_pose.pose.position.x
        goal_y = goal_pose.pose.position.y

        # Calculate distance and angle to goal
        dist_to_goal = sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)
        angle_to_goal = atan2(goal_y - current_y, goal_x - current_x)
        
        angle_error = angle_to_goal - current_yaw
        # Normalize the angle error to be between -pi and pi
        if angle_error > pi:
            angle_error -= 2 * pi
        elif angle_error < -pi:
            angle_error += 2 * pi

        # Proportional controller
        if dist_to_goal > 0.1: # Goal tolerance
            if abs(angle_error) > 0.1:
                # Turn first, then move forward
                twist_msg.angular.z = 0.4 * angle_error
            else:
                twist_msg.linear.x = 0.15 # Move forward
        else:
            self.get_logger().info('Goal reached!')
            self.path = [] # Clear the path

        self.cmd_vel_pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()