import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    launch_dir = os.path.dirname(os.path.realpath(__file__))

    declared_args = [
        DeclareLaunchArgument('cloud_topic', default_value='/rslidar/points'),
        DeclareLaunchArgument('odometry_topic', default_value='/graph_msf/est_odometry_odom_imu'),
        DeclareLaunchArgument('pose_stamped_topic', default_value='no_pose_stamped_topic'),
        DeclareLaunchArgument('pose_stamped_with_covariance_topic', default_value='empty'),
        DeclareLaunchArgument('launch_prefix', default_value=''),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('distance_cutoff', default_value='0.2'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value='/opt/ros/jazzy/share/rviz2/rviz/default_config.rviz'),

        # Additional args for smb_estimator_graph_ros2
        DeclareLaunchArgument('imu_topic_name', default_value='/imu/data_raw', description='IMU topic name'),
        DeclareLaunchArgument('lidar_odometry_topic_name', default_value='/dlio/odom_node/odom', description='Lidar odometry topic name'),
        DeclareLaunchArgument('wheel_odometry_topic_name', default_value='/control/smb_diff_drive/odom', description='Wheel odometry topic name'),
        DeclareLaunchArgument('wheel_velocities_topic_name', default_value='/control/smb_lowlevel_controller/wheelSpeeds', description='Wheel velocities topic name'),
        DeclareLaunchArgument('vio_odometry_topic_name', default_value='/tracking_camera/odom/sample', description='VIO odometry topic name'),
        # For logging_dir_location, let the included launch handle its default if not passed
    ]

    # Include open3d_launch.py
    open3d_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'open3d_launch.py')
        ),
        launch_arguments={
            'cloud_topic': LaunchConfiguration('cloud_topic'),
            'odometry_topic': LaunchConfiguration('odometry_topic'),
            'launch_prefix': LaunchConfiguration('launch_prefix'),
            'launch_rviz': LaunchConfiguration('launch_rviz'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            # Add others as needed
        }.items(),
    )

    # Include robosense_processor_launch.py
    robosense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'robosense_processor_launch.py')
        ),
        launch_arguments={
            'cloud_topic': LaunchConfiguration('cloud_topic'),
            'pose_topic': LaunchConfiguration('odometry_topic'),
            'distance_cutoff': LaunchConfiguration('distance_cutoff'),
            'launch_prefix': LaunchConfiguration('launch_prefix'),
            'launch_rviz': LaunchConfiguration('launch_rviz'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'rviz_config': LaunchConfiguration('rviz_config'),
        }.items(),
    )

    # Include smb_estimator_graph_sim.launch.py from smb_estimator_graph_ros2 package
    smb_estimator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('smb_estimator_graph_ros2'),
                'launch',
                'smb_estimator_graph_sim.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'imu_topic_name': LaunchConfiguration('imu_topic_name'),
            'lidar_odometry_topic_name': LaunchConfiguration('lidar_odometry_topic_name'),
            'wheel_odometry_topic_name': LaunchConfiguration('wheel_odometry_topic_name'),
            'wheel_velocities_topic_name': LaunchConfiguration('wheel_velocities_topic_name'),
            'vio_odometry_topic_name': LaunchConfiguration('vio_odometry_topic_name'),
            # 'logging_dir_location': LaunchConfiguration('logging_dir_location'),  # Uncomment if you want to expose this
        }.items(),
    )

    return LaunchDescription(
        declared_args + [
            open3d_launch,
            robosense_launch,
            smb_estimator_launch,
        ]
    )
