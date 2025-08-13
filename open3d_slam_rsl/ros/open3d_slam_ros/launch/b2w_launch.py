import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    launch_dir = os.path.dirname(os.path.realpath(__file__))

    declared_args = [
        DeclareLaunchArgument('cloud_topic', default_value='/lidar/point_cloud'), # lidar frame
        DeclareLaunchArgument('odometry_topic', default_value='/graph_msf/est_odometry_odom_imu'),
        DeclareLaunchArgument('assumed_external_odometry_tracked_frame', default_value='imu'), #imu_link #base_link
        DeclareLaunchArgument('pose_stamped_topic', default_value='no_pose_stamped_topic'),
        DeclareLaunchArgument('pose_stamped_with_covariance_topic', default_value='empty'),
        DeclareLaunchArgument('launch_prefix', default_value=''),
        DeclareLaunchArgument('launch_rviz', default_value='false   '),
        DeclareLaunchArgument('distance_cutoff', default_value='0.2'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value='/opt/ros/jazzy/share/rviz2/rviz/default_config.rviz'),
        DeclareLaunchArgument('imu_topic_name', default_value='/imu_sensor_broadcaster/imu', description='IMU topic name'),
        DeclareLaunchArgument('lidar_odometry_topic_name', default_value='/graph_msf/est_odometry_odom_imu', description='Lidar odometry topic name'),
        DeclareLaunchArgument('parameter_filename', default_value='param_b2w.lua'),
    ]


    open3d_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'open3d_b2w_launch.py')
        ),
        launch_arguments={
            'cloud_topic': LaunchConfiguration('cloud_topic'),
            'odometry_topic': LaunchConfiguration('odometry_topic'),
            'launch_prefix': LaunchConfiguration('launch_prefix'),
            'launch_rviz': LaunchConfiguration('launch_rviz'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'parameter_filename': LaunchConfiguration('parameter_filename'),
            
        }.items(),
    )


    return LaunchDescription(
        declared_args + [
            open3d_launch,
        ]
    )
