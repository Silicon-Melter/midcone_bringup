import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
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

    # Replaced 'px4_device' with 'fcu_url' for MAVROS
    # Common Baud rates: 57600 (Telemetry), 921600 (USB/High speed), 115200 (Standard)
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url', default_value='/dev/ttyACM0:115200', 
        description='Serial device and baud rate for MAVROS'
    )

    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url', default_value='udp://@',
        description='GCS Bridge URL (allows QGroundControl to connect)'
    )
    
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/ttyUSB0',
        description='Serial port for RPLidar'
    )

    # --- URDF SETUP ---
    pkg_share_bringup = FindPackageShare('midcone_bringup')
    urdf_path = PathJoinSubstitution([pkg_share_bringup, 'urdf', LaunchConfiguration('model')])
    robot_desc_content = Command(['cat ', urdf_path])

    # --- MAVROS CONFIGURATION ---
    # We load the config file dynamically. 
    # NOTE: If using ArduPilot, change 'px4_config.yaml' to 'apm_config.yaml'
    mavros_config = PathJoinSubstitution([
        FindPackageShare('mavros'), 'launch', 'px4_config.yaml'
    ])

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

    # 3. MAVROS Node (Replaces MicroXRCEAgent)
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        parameters=[
            mavros_config, # Load the config file
            {
                'fcu_url': LaunchConfiguration('fcu_url'),
                'gcs_url': LaunchConfiguration('gcs_url'),
                'target_system_id': 1,
                'target_component_id': 1,
                'fcu_protocol': 'v2.0' # Force MAVLink v2
            }
        ]
    )

    # 4. RPLidar (Direct Node - Forced Standard Mode)
    # Kept direct to prevent "Scan mode Sensitivity" errors
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('lidar_port'),
            'serial_baudrate': 115200,
            'frame_id': 'lidar_link',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard'
        }]
    )

    # 5. RealSense Camera (Direct Node - Low Bandwidth Config)
    # Kept direct to prevent "Buffer Exceeded" crashes
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        output='screen',
        parameters=[{
            'camera_name': 'camera',
            
            # --- CRITICAL STABILITY SETTINGS ---
            'global_time_enabled': False,
            
            # Low Res + Low FPS + No Infra = Stable on Drone
            'depth_module.profile': '1280x720x10',
            'rgb_camera.profile': '1280x720x10',
            'enable_infra1': True,
            'enable_infra2': True,

            'pointcloud.enable': True,
            'align_depth.enable': True
        }]
    )

    return LaunchDescription([
        sim_time_arg,
        model_arg,
        fcu_url_arg,
        gcs_url_arg,
        lidar_port_arg,
        rsp_node,
        jsp_node,
        mavros_node,
        rplidar_node,
        realsense_node
    ])