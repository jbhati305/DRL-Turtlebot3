#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time

class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender')
        
        # Publisher for goals
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Timer to send test goals
        self.timer = self.create_timer(5.0, self.send_test_goal)
        self.goal_count = 0
        
        self.get_logger().info("Goal Sender initialized")
        
    def send_test_goal(self):
        """Send a test goal"""
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        
        # Define test goals
        test_goals = [
            (2.0, 2.0),   # Goal 1
            (-2.0, 2.0),  # Goal 2
            (-2.0, -2.0), # Goal 3
            (2.0, -2.0),  # Goal 4
            (0.0, 0.0),   # Back to start
        ]
        
        if self.goal_count < len(test_goals):
            x, y = test_goals[self.goal_count]
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.position.z = 0.0
            goal.pose.orientation.w = 1.0
            
            self.goal_pub.publish(goal)
            self.get_logger().info(f"Sent goal {self.goal_count + 1}: ({x}, {y})")
            self.goal_count += 1
        else:
            self.get_logger().info("All test goals sent")
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = GoalSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 