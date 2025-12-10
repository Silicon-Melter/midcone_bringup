import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # --- ARGUMENTS ---
    
    # 1. Sim Time (Default False for hardware)
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='false',
        description='Use simulation (bag) time'
    )
    
    # 2. URDF Model (Default drone.urdf)
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='drone.urdf',
        description='Name of the urdf file in the bringup/urdf directory'
    )

    # 3. PX4 Device Port (Default /dev/ttyUSB0)
    px4_device_arg = DeclareLaunchArgument(
        'px4_device',
        default_value='/dev/ttyUSB0',
        description='Serial port for MicroXRCEAgent'
    )

    # 4. Lidar Launch File (Default view_rplidar_a1_launch.py)
    # Tip: Change this to 'rplidar_a1_launch.py' if you don't want Rviz to open automatically
    lidar_launch_arg = DeclareLaunchArgument(
        'lidar_launch_file',
        default_value='view_rplidar_a1_launch.py',
        description='The launch file to run from rplidar_ros package'
    )

    # --- URDF CONFIGURATION ---
    pkg_share = FindPackageShare('bringup')
    
    urdf_path = PathJoinSubstitution([
        pkg_share,
        'urdf',
        LaunchConfiguration('model')
    ])

    robot_desc_content = Command(['cat ', urdf_path])


    # --- NODES ---

    # 1. Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': robot_desc_content
        }]
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
    # Uses the 'px4_device' argument
    px4_agent_cmd = ExecuteProcess(
        cmd=[
            'MicroXRCEAgent', 'serial', 
            '-D', LaunchConfiguration('px4_device')
        ],
        output='screen'
    )

    # 4. RPLidar
    # Uses the 'lidar_launch_file' argument
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('rplidar_ros'),
                'launch',
                LaunchConfiguration('lidar_launch_file')
            ])
        ),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    return LaunchDescription([
        #arguments
        sim_time_arg,
        model_arg,
        px4_device_arg,
        lidar_launch_arg,
        #nodes
        rsp_node,
        jsp_node,
        px4_agent_cmd,
        rplidar_launch
    ])