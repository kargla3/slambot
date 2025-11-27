from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
import xacro
import os
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context)

    share_dir = get_package_share_directory('slambot_description')
    xacro_file = os.path.join(share_dir, 'urdf', 'mapping_robot_v1.xacro')
    config_dir = os.path.join(share_dir, 'config', 'controllers.yaml')
    slam_params_file = os.path.join(share_dir, 'config', 'slam_toolbox.yaml')

    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    nodes_to_launch = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {'robot_description': robot_urdf}
            ]
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher'
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                {"robot_description": robot_urdf},
                config_dir
            ],
            output="screen",
            remappings=[('/diff_cont/cmd_vel_unstamped', '/cmd_vel')],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diff_cont", "--controller-manager", "/controller_manager"],
        ),
        Node(
            package="scanners_driver",
            executable="lidar_driver",
            name="lidar_driver"
        ),
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
                slam_params_file,
                {'mode': mode}
            ],
            output='screen'
        ),
    ]
    return nodes_to_launch

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mapping', description='Mode for slam_toolbox: mapping or localization'),
        OpaqueFunction(function=launch_setup)
    ])
