from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg = get_package_share_directory('robot_zero')

    urdf = os.path.join(pkg, 'urdf', 'robot_core.xacro')
    controller_yaml = os.path.join(pkg, 'config', 'controllers.yaml')

    robot_description = Command(['xacro ', urdf])

    # 1. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # 2. ros2_control_node (THIS loads your hardware plugin)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {'robot_description': robot_description},
            controller_yaml
        ],
        output="screen"
    )

    # 3. Spawn controllers
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager"
        ],
        output="screen"
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--controller-manager", "/controller_manager"
        ],
        output="screen"
    )

    return LaunchDescription([
        robot_state_publisher,
        control_node,
        joint_state_spawner,
        diff_drive_spawner
    ])