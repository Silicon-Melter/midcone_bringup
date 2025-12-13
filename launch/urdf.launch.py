import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # --- ARGUMENTS ---
    # 1. Declare Arguments
    # 'use_sim_time': Allows syncing with Bag files. 
    # Default is 'true' based on your snippet, but for real hardware you usually want 'false'.
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='true',
        description='Use simulation (bag) time'
    )
    
    # 'model': The filename input (Default: drone.urdf)
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='handheld.urdf',
        description='Name of the urdf file in the bringup/urdf directory'
    )

    # --- URDF CONFIGURATION ---
    # 2. Dynamic Path Construction
    # Finds: install/bringup/share/bringup/urdf/<model_name>
    pkg_share = FindPackageShare('midcone_bringup')
    
    urdf_path = PathJoinSubstitution([
        pkg_share,
        'urdf',
        LaunchConfiguration('model')
    ])

    # 3. Read the URDF (Command)
    # We use 'cat' combined with the dynamic path
    robot_desc_content = Command(['cat ', urdf_path])

    # --- NODES ---

    # 4. Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': robot_desc_content
        }]
    )

    # 5. Joint State Publisher
    # We pass the path as an ARGUMENT so it reads the file immediately
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        arguments=[urdf_path],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )


    return LaunchDescription([
        sim_time_arg,
        model_arg,
        rsp_node,
        jsp_node,
    ])