import os
import platform
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution


def generate_launch_description():

    world_arg = DeclareLaunchArgument(
        'world', default_value='empty.sdf',
        description='Name of the Gazebo world file to load'
    )

    pkg_tb3_simulation = get_package_share_directory('tb3_simulation')

    # Add your own gazebo library path here
    gazebo_models_path = "/home/david/gazebo_models"
    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + gazebo_models_path

    # Get the world file path
    world_file_path = PathJoinSubstitution([
        pkg_tb3_simulation,
        'worlds',
        LaunchConfiguration('world')
    ])

    # Check if we're on macOS
    is_macos = platform.system() == "Darwin"

    if is_macos:
        # On macOS, we need to run server and GUI separately
        gazebo_server = ExecuteProcess(
            cmd=['gz', 'sim', '-s', '-r', '-v', '-v1', '--render-engine', 'ogre2', 
                 TextSubstitution(text=' '), world_file_path],
            output='screen',
            shell=True
        )

        # # Launch GUI after a short delay to ensure server is ready
        # gazebo_gui = TimerAction(
        #     period=2.0,
        #     actions=[
        #         ExecuteProcess(
        #             cmd=['gz', 'sim', '-g'],
        #             output='screen',
        #             shell=True
        #         )
        #     ]
        # )

        launchDescriptionObject = LaunchDescription()
        launchDescriptionObject.add_action(world_arg)
        launchDescriptionObject.add_action(gazebo_server)
        # launchDescriptionObject.add_action(gazebo_gui)

    else:
        # On Linux, use the standard approach
        from launch.actions import IncludeLaunchDescription
        from launch.launch_description_sources import PythonLaunchDescriptionSource
        
        pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
        
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
            ),
            launch_arguments={'gz_args': [world_file_path,
                TextSubstitution(text=' -s -r -v -v1 --render-engine ogre2')],
                'on_exit_shutdown': 'true'}.items()
        )

        launchDescriptionObject = LaunchDescription()
        launchDescriptionObject.add_action(world_arg)
        launchDescriptionObject.add_action(gazebo_launch)

    return launchDescriptionObject