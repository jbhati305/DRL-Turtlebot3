# drl_navigaion/drl_navigaion/global_planner.py

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer

class GlobalPlanner(Node):
    def __init__(self):
        super().__init__('global_planner_node')
        self.get_logger().info('Global Planner Node Started')

        # Create publisher for the path
        self.path_pub = self.create_publisher(Path, '/global_path', 10)

        # Create subscriber for the goal
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        # TF listener to get robot's current pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def goal_callback(self, msg: PoseStamped):
        self.get_logger().info('Received a new goal!')
        
        # Get the current pose of the robot
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
            
            start_pose = PoseStamped()
            start_pose.header.frame_id = 'map'
            start_pose.header.stamp = self.get_clock().now().to_msg()
            start_pose.pose.position.x = transform.transform.translation.x
            start_pose.pose.position.y = transform.transform.translation.y
            start_pose.pose.orientation = transform.transform.rotation
            
            self.get_logger().info('Planning path...')
            self.plan_path(start_pose, msg)

        except Exception as e:
            self.get_logger().error(f'Could not get robot pose: {e}')

    def plan_path(self, start_pose: PoseStamped, goal_pose: PoseStamped):
        # This is where a real path planning algorithm like A* would go.
        # For simplicity, we'll create a direct, straight-line path.
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        
        path.poses.append(start_pose)
        path.poses.append(goal_pose) # Add the final goal

        self.path_pub.publish(path)
        self.get_logger().info('Simple path published!')

def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()