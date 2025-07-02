import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def check_required_packages(pkgs):
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    missing = []
    for pkg in pkgs:
        try:
            get_package_share_directory(pkg)
        except PackageNotFoundError:
            missing.append(pkg)
    if missing:
        warn = (
            "\033[1;31m"
            + "WARNING: Required ROS 2 packages missing: "
            + ", ".join(missing)
            + "\nInstall them with:\n    sudo apt install ros-${ROS_DISTRO}-"
            + " ros-${ROS_DISTRO}-".join(missing)
            + "\033[0m"
        )
        print(warn, file=sys.stderr)
    return missing

def launch_ros2_bag_play(context, *args, **kwargs):
    play_bag = LaunchConfiguration('play_bag').perform(context)
    if play_bag.lower() not in ['1', 'true', 'yes']:
        return []
    bag_path = LaunchConfiguration('bag_path').perform(context)
    cmd = [
        'ros2', 'bag', 'play', bag_path, '--clock', '--remap', '/tf:=/old_tf'
    ]
    return [
        ExecuteProcess(
            cmd=cmd,
            output='screen'
        )
    ]

def generate_launch_description():
    check_required_packages([
        "open3d_slam_ros",
    ])

    launch_dir = os.path.dirname(os.path.realpath(__file__))

    declared_args = [
        DeclareLaunchArgument('cloud_topic', default_value='/lidar/point_cloud'), # lidar frame
        DeclareLaunchArgument('odometry_topic', default_value='/graph_msf/est_odometry_odom_imu'),
        DeclareLaunchArgument('pose_stamped_topic', default_value='no_pose_stamped_topic'),
        DeclareLaunchArgument('pose_stamped_with_covariance_topic', default_value='empty'),
        DeclareLaunchArgument('launch_prefix', default_value=''),
        DeclareLaunchArgument('launch_rviz', default_value='false   '),
        DeclareLaunchArgument('distance_cutoff', default_value='0.2'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value='/opt/ros/jazzy/share/rviz2/rviz/default_config.rviz'),
        # Arguments for smb_estimator_graph_ros2
        DeclareLaunchArgument('imu_topic_name', default_value='/imu_sensor_broadcaster/imu', description='IMU topic name'),
        DeclareLaunchArgument('lidar_odometry_topic_name', default_value='/open3d/scan2map_odometry', description='Lidar odometry topic name'),
        DeclareLaunchArgument('wheel_odometry_topic_name', default_value='/control/smb_diff_drive/odom', description='Wheel odometry topic name'),
        DeclareLaunchArgument('wheel_velocities_topic_name', default_value='/control/smb_lowlevel_controller/wheelSpeeds', description='Wheel velocities topic name'),
        DeclareLaunchArgument('vio_odometry_topic_name', default_value='/tracking_camera/odom/sample', description='VIO odometry topic name'),
        # Bag play options
        DeclareLaunchArgument('bag_path', default_value='/home/ttuna/colcon_ws/src/smb_bag_wheels_5.mcap', description='Bag file to play'),
        DeclareLaunchArgument('play_bag', default_value='true', description='Launch ros2 bag play (true/false)'),
        DeclareLaunchArgument('parameter_filename', default_value='param_b2w.lua'),
    ]

    description_file = PathJoinSubstitution(
        [FindPackageShare("smb_description"), "urdf", "smb.urdf.xacro"]
    )

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
        }.items(),
    )

    # bag_play_action = OpaqueFunction(function=launch_ros2_bag_play)

    return LaunchDescription(
        declared_args + [
            open3d_launch,
            # smb_estimator_launch,
            # bag_play_action,
        ]
    )
