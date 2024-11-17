import os
from ament_index_python.packages import get_package_share_directory
import launch_ros
from launch_ros.actions import Node
from launch import LaunchDescription 
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    # Keep existing configurations
    use_sim_time = LaunchConfiguration("use_sim_time")
    description_path = LaunchConfiguration("description_path")
    
    # Get package paths
    config_pkg_share = get_package_share_directory("champ_config")
    descr_pkg_share = get_package_share_directory("champ_description")
    ros_gz_sim_dir = get_package_share_directory("ros_gz_sim")
    
    # Load configurations
    joints_config = os.path.join(config_pkg_share, "config/joints/joints.yaml")
    ros_control_config = os.path.join(config_pkg_share, "config/ros_control/ros_control.yaml")
    gait_config = os.path.join(config_pkg_share, "config/gait/gait.yaml")
    links_config = os.path.join(config_pkg_share, "config/links/links.yaml")
    default_model_path = os.path.join(descr_pkg_share, "urdf/champ.urdf.xacro")
    default_world_path = os.path.join(config_pkg_share, "worlds/default.world")

    # Keep existing launch arguments declarations
    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="true")
    declare_rviz = DeclareLaunchArgument("rviz", default_value="false")
    declare_robot_name = DeclareLaunchArgument("robot_name", default_value="champ")
    declare_lite = DeclareLaunchArgument("lite", default_value="false")
    declare_world = DeclareLaunchArgument("world", default_value=default_world_path)
    declare_gui = DeclareLaunchArgument("gui", default_value="true")
    declare_world_init_x = DeclareLaunchArgument("world_init_x", default_value="0.0")
    declare_world_init_y = DeclareLaunchArgument("world_init_y", default_value="0.0")
    declare_world_init_heading = DeclareLaunchArgument("world_init_heading", default_value="0.6")

    # Start Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-v 4 -r ' + LaunchConfiguration('world'),
            'on_exit_shutdown': 'true'
        }.items()
    )

    # Spawn Robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', LaunchConfiguration('robot_name'),
            '-x', LaunchConfiguration('world_init_x'),
            '-y', LaunchConfiguration('world_init_y'),
            '-z', '0.5',
            '-Y', LaunchConfiguration('world_init_heading'),
            '-file', default_model_path,
        ]
    )

    # Keep existing bringup launch
    bringup_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("champ_bringup"), "launch", "bringup.launch.py")
        ),
        launch_arguments={
            "description_path": default_model_path,
            "joints_map_path": joints_config,
            "links_map_path": links_config,
            "gait_config_path": gait_config,
            "use_sim_time": use_sim_time,
            "robot_name": LaunchConfiguration("robot_name"),
            "gazebo": "true",
            "lite": LaunchConfiguration("lite"),
            "rviz": LaunchConfiguration("rviz"),
            "joint_controller_topic": "joint_group_effort_controller/joint_trajectory",
            "hardware_connected": "false",
            "publish_foot_contacts": "false",
            "close_loop_odom": "true",
        }.items(),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_rviz,
        declare_robot_name,
        declare_lite,
        declare_world,
        declare_gui,
        declare_world_init_x,
        declare_world_init_y,
        declare_world_init_heading,
        gazebo,
        spawn_robot,
        bringup_ld
    ])
