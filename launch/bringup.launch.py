import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # --- ARGUMENTS ---
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (bag) time'
    )
    
    model_arg = DeclareLaunchArgument(
        'model', default_value='drone.urdf',
        description='Name of the urdf file'
    )

    px4_device_arg = DeclareLaunchArgument(
        'px4_device', default_value='/dev/ttyAMA0',
        description='Serial port for MicroXRCEAgent'
    )
    
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/ttyUSB0',
        description='Serial port for RPLidar'
    )

    # --- URDF SETUP ---
    pkg_share_bringup = FindPackageShare('midcone_bringup')
    urdf_path = PathJoinSubstitution([pkg_share_bringup, 'urdf', LaunchConfiguration('model')])
    robot_desc_content = Command(['cat ', urdf_path])

    # 1. Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                     'robot_description': robot_desc_content}]
    )

    # 2. Joint State Publisher
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        arguments=[urdf_path],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # 3. MicroXRCEAgent (PX4)
    px4_agent_cmd = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'serial', '-D', LaunchConfiguration('px4_device')],
        output='screen'
    )

    # 4. RPLidar (Using rplidar_a1_launch.py)
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rplidar_ros'),
                'launch',
                'rplidar_a1_launch.py'
            ])
        ),
        launch_arguments={
            'serial_port': LaunchConfiguration('lidar_port'),
            'frame_id': 'lidar_link',
            'angle_compensate': 'true',
            'scan_mode': 'Standard' 
        }.items()
    )

    # 5. RealSense Camera (Using rs_launch.py)
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ),
        launch_arguments={
            'camera_name': 'camera',
            'pointcloud.enable': 'true',
            'align_depth.enable': 'true'
        }.items()
    )

    return LaunchDescription([
        sim_time_arg,
        model_arg,
        px4_device_arg,
        lidar_port_arg,
        rsp_node,
        jsp_node,
        px4_agent_cmd,
        lidar_launch,
        realsense_launch
    ])