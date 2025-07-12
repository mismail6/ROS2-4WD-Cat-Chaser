from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ros_gz_bridge.actions import RosGzBridge
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    urdf_file = PathJoinSubstitution([
        FindPackageShare("cat_chaser"),
        "urdf",
        "my_robot.urdf.xacro"
    ])

    bridge_config_file = PathJoinSubstitution([
        FindPackageShare("cat_chaser"),  # Your package name
        "config",
        "bridge_config.yaml"
    ])

    rviz_config = PathJoinSubstitution([
        FindPackageShare("cat_chaser"),
        "rviz",
        "hiwonder_config.rviz"
    ])

    robot_description = Command(["xacro ", urdf_file])

    return LaunchDescription([
        # Publishes the robot_description
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="hiwonder_state_publisher",
            output="screen",
            parameters=[
                {"robot_description": robot_description},
                {"use_sim_time": True}
            ]
        ),

        # Spawn entity in Gazebo using ros_gz_sim
        Node(
            package="ros_gz_sim",
            executable="create",
            name="spawn_robot",
            arguments=[
                "-name", "hiwonder",
                "-topic", "robot_description",
                "-x", "0", "-y", "0", "-z", "0.4"
            ],
            output="screen"
        ),
        RosGzBridge(
            bridge_name='my_bridge',
            config_file=bridge_config_file,
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_base_link",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            parameters=[{"use_sim_time": True}]
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            parameters=[{"use_sim_time": True}],
            arguments=["-d", rviz_config]
        ),
        Node(
            package="cat_chaser",
            executable="odom_to_base",
            name="odom_to_base",
            output="screen",
            parameters=[{"use_sim_time": True}]
        ),
    ])
