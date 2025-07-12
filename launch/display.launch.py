#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package Directories
    pkg_share = FindPackageShare(package='cat_chaser').find('cat_chaser')
    
    # Paths
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/display.rviz')
    
    # Launch configuration variables
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare the launch arguments
    declare_rviz_config_cmd = DeclareLaunchArgument(
        name='rviz_config',
        default_value=default_rviz_config_path,
        description='Absolute path to rviz config file')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    # Bridge joint states from Gazebo to ROS2 
    gz_joint_state_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_joint_state_bridge',
        output='screen',
        arguments=['/world/default/model/hiwonder/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'],
        remappings=[('/world/default/model/hiwonder/joint_state', '/joint_states')],
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )
    
    # Bridge odometry from Gazebo to ROS2 (this should publish TF automatically)
    gz_odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_odom_bridge',
        output='screen',
        arguments=['/model/hiwonder/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
        remappings=[('/model/hiwonder/odometry', '/odom')],
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )
    
    # Static Transform: map → hiwonder/odom (to match your odometry frame_id)
    static_transform_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_map_odom',
        output='screen',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'map', '--child-frame-id', 'hiwonder/odom'],
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )
    
    # Static Transform: hiwonder/base_link → base_link (bridge frame naming)
    static_transform_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_bridge',  
        output='screen',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0', '--yaw', '0', '--frame-id', 'hiwonder/base_link', '--child-frame-id', 'base_link'],
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )
    
    # RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if rviz_config != default_rviz_config_path else [],
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    
    # Add only the nodes needed for RViz (robot_state_publisher already running from Gazebo launch)
    ld.add_action(gz_joint_state_bridge)
    ld.add_action(gz_odom_bridge)
    ld.add_action(static_transform_map_odom)
    ld.add_action(static_transform_bridge)
    ld.add_action(rviz_node)
    
    return ld