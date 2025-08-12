from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="calculate_robot_speed",
                executable="rpm_publisher",
                name="rpm_node",
                parameters=[{"rpm_value": 5.0}],
            ),
            Node(
                package="calculate_robot_speed",
                executable="speed_publisher",
                name="speed_node",
                parameters=[{"wheel_radius": 10.0 / 100.0}],
            ),
            ExecuteProcess(
                cmd=["ros2", "topic", "echo", "/robot_speed"],
                output="screen",
            ),
        ]
    )
