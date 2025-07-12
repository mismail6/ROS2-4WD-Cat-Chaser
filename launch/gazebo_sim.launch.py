from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    urdf_file = PathJoinSubstitution([
        FindPackageShare("cat_chaser"),
        "urdf",
        "my_robot.urdf.xacro"
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
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                       '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
                       '/model/hiwonder/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'], 
                       # model/hiwonder/odometry is same topic for ros2 so it defaults to it. Otherwise, we could add @/odom to clearly specific the ros2 topic name as well
            output='screen'
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            parameters=[{"use_sim_time": True}]
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            parameters=[{"use_sim_time": True}]
        ),
    ])
