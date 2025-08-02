import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_drl_navigation = get_package_share_directory('drl_navigation')
    pkg_tb3_simulation = get_package_share_directory('tb3_simulation')

    # Declare arguments
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Use simulation time'
    )

    # Include simulation launch
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_simulation, 'launch', 'spawn_robot.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'rviz': 'false',
        }.items()
    )

    # Launch RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_drl_navigation, 'rviz', 'custom_navigation.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen'
    )

    # Launch RViz with delay
    rviz_timer = TimerAction(
        period=5.0,
        actions=[rviz_node]
    )

    # Launch map server to load existing map
    map_server_node = Node(
        package='drl_navigation',
        executable='map_server',
        name='map_server',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen'
    )

    # Launch our custom global planner
    global_planner_node = Node(
        package='drl_navigation',
        executable='global_planner',
        name='global_planner',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen'
    )

    # Launch our custom controller
    controller_node = Node(
        package='drl_navigation',
        executable='controller',
        name='controller',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen'
    )

    # Launch goal sender (optional)
    goal_sender_node = Node(
        package='drl_navigation',
        executable='goal_sender',
        name='goal_sender',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen'
    )

    # Launch nodes with timing
    map_timer = TimerAction(
        period=2.0,
        actions=[map_server_node]
    )

    nav_timer = TimerAction(
        period=4.0,
        actions=[
            global_planner_node,
            controller_node,
            goal_sender_node
        ]
    )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(rviz_arg)
    launchDescriptionObject.add_action(sim_time_arg)
    launchDescriptionObject.add_action(simulation_launch)
    launchDescriptionObject.add_action(rviz_timer)
    launchDescriptionObject.add_action(map_timer)
    launchDescriptionObject.add_action(nav_timer)

    return launchDescriptionObject 