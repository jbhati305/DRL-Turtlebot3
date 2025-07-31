import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_tb3_simulation = get_package_share_directory("tb3_simulation")
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Set up Gazebo environment variables
    gazebo_models_path, ignore_last_dir = os.path.split(pkg_tb3_simulation)
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path

    # Launch arguments
    rviz_launch_arg = DeclareLaunchArgument(
        "rviz", default_value="false", description="Open RViz"
    )

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config", default_value="m4_stable.rviz", description="RViz config file"
    )

    world_arg = DeclareLaunchArgument(
        "world",
        default_value="empty.sdf",
        description="Name of the Gazebo world file to load",
    )

    model_arg = DeclareLaunchArgument(
        "model",
        default_value="tb3_waffle.urdf",
        description="Name of the URDF description to load",
    )

    x_arg = DeclareLaunchArgument(
        "x", default_value="0.0", description="x coordinate of spawned robot"
    )

    y_arg = DeclareLaunchArgument(
        "y", default_value="0.0", description="y coordinate of spawned robot"
    )

    z_arg = DeclareLaunchArgument(
        "z", default_value="0.5", description="z coordinate of spawned robot"
    )

    yaw_arg = DeclareLaunchArgument(
        "yaw", default_value="0.0", description="yaw angle of spawned robot"
    )

    robot_name_arg = DeclareLaunchArgument(
        "robot_name", default_value="my_robot", description="Name of the spawned robot"
    )

    sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="True", description="Flag to enable use_sim_time"
    )

    gui_arg = DeclareLaunchArgument(
        "gui", default_value="true", description="Start Gazebo GUI"
    )

    # Define the path to your URDF file
    urdf_file_path = PathJoinSubstitution(
        [
            pkg_tb3_simulation,
            "urdf",
            LaunchConfiguration("model"),
        ]
    )

    # Start Gazebo simulation (headless)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={
            'gz_args': [PathJoinSubstitution([
                pkg_tb3_simulation,
                'worlds',
                LaunchConfiguration('world')
            ]), ' -s -r'],  # -s for server mode (headless), -r to start running
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Start Gazebo GUI separately (if requested)
    gazebo_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        output='screen',
        condition=IfCondition(LaunchConfiguration("gui"))
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": Command(["xacro", " ", urdf_file_path]),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    # Launch RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [pkg_tb3_simulation, "rviz", LaunchConfiguration("rviz_config")]
            ),
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        output="screen",
    )

    # Spawn robot using gz service (with delay to ensure Gazebo is ready)
    spawn_robot = ExecuteProcess(
        cmd=[
            'gz', 'service', '-s', '/world/empty/create',
            '--reqtype', 'gz.msgs.EntityFactory',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '1000',
            '--req', [
                'sdf_filename: "', urdf_file_path, '", ',
                'name: "', LaunchConfiguration("robot_name"), '", ',
                'pose: {position: {x: ', LaunchConfiguration("x"), ', y: ', LaunchConfiguration("y"), ', z: ', LaunchConfiguration("z"), '}, ',
                'orientation: {x: 0, y: 0, z: 0, w: 1}}'
            ]
        ],
        output="screen",
    )

    # Add timer to delay robot spawning (give Gazebo time to start)
    delayed_spawn = TimerAction(
        period=3.0,
        actions=[spawn_robot]
    )

    # Node to bridge topics between Gazebo and ROS2
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry]gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState]gz.msgs.Model",
            "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",  # Add TF bridge

            "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
            "/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
            "/navsat@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat",
            "/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
        ],
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    # Image bridge for camera topics
    gz_image_bridge_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            "/camera/image",
        ],
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "camera.image.compressed.jpeg_quality": 75,
            },
        ],
    )

    return LaunchDescription([
        # Launch arguments
        rviz_launch_arg,
        rviz_config_arg,
        world_arg,
        model_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        robot_name_arg,
        sim_time_arg,
        gui_arg,
        
        # Launch nodes and processes
        gazebo_launch,
        robot_state_publisher_node,
        rviz_node,
        delayed_spawn,
        gazebo_gui,
        gz_bridge_node,
        gz_image_bridge_node,
    ]) 