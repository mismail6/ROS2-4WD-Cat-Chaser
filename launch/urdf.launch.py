from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    urdf_path = PathJoinSubstitution([
        FindPackageShare("cat_chaser"),
        "urdf",
        "my_robot.urdf"
    ])

    return LaunchDescription([
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen"
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": urdf_path
            }]
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen"
        )
    ])
