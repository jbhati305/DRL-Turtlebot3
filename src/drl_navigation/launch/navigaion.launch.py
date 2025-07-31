import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Get path to your package and nav2_bringup config files
    drl_navigation_dir = get_package_share_directory('drl_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Path to slam_toolbox launch files (or config files if you prefer direct node)
    slam_toolbox_launch_dir = get_package_share_directory('slam_toolbox')


    # Declare arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Use simulation (Gazebo) clock if true')

    # SLAM Toolbox Node (instead of map_server and amcl)
    # You can either include the standard launch file or define the node directly
    # Including the launch file is often easier as it sets up parameters.
    # We'll use online_async_launch.py for asynchronous SLAM (good for real-time).
    slam_toolbox_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_toolbox_launch_dir, 'launch', 'online_async_launch.py')),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )
    
    # You might need to provide custom parameters for slam_toolbox if the defaults
    # don't work well for your robot/sensor.
    # Example:
    # slam_config_file = os.path.join(drl_navigation_dir, 'config', 'slam_params.yaml')
    # slam_toolbox_node = Node(
    #     package='slam_toolbox',
    #     executable='async_slam_toolbox_node', # Or 'sync_slam_toolbox_node'
    #     name='slam_toolbox',
    #     output='screen',
    #     parameters=[slam_config_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    # )


    # Your custom nodes
    global_planner_node = Node(
        package='drl_navigation',
        executable='global_planner',
        name='global_planner_node',
        output='screen',
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
        # Ensure this node is subscribed to /map and /tf from slam_toolbox
        # and publishes to cmd_vel or similar for robot movement.
    )

    controller_node = Node(
        package='drl_navigation',
        executable='controller',
        name='controller_node',
        output='screen',
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
        # This node will likely take local plans from a local planner (if you have one)
        # and convert them to robot commands.
        # It also needed the transforms3d library, so ensure that's installed.
    )

    return LaunchDescription([
        use_sim_time,
        
        # Replace map_server/amcl/lifecycle_manager_localization with slam_toolbox
        slam_toolbox_node, 

        global_planner_node,
        controller_node,
    ])