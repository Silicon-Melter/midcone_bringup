import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # --- 1. OPTIMIZED ODOMETRY PARAMETERS ---
    parameters=[{
          'frame_id':'base_link', 
          'subscribe_depth':True,
          'subscribe_odom_info':True,
          'approx_sync':False,
          
          'Odom/Strategy': '1', 
          'Reg/Force3DoF': 'False',
          'Odom/KalmanUpdate': 'True',
          'Odom/KalmanProcessNoise': '0.001', 
          'Odom/KalmanMeasurementNoise': '0.01',
          
          'Vis/FeatureType': '8',
          'Vis/MaxFeatures': '800',
          'Vis/MinInliers': '20',
          'GFTT/MinDistance': '10', 
          
          # 'Odom/GuessMotion': 'True'
    }]

    remappings=[
          # Inputs
          ('imu', '/mavros/imu/data'),      # If you want to use PX4 IMU for Odom guess
          ('rgb/image', '/camera/camera/color/image_raw'),
          ('rgb/camera_info', '/camera/camera/aligned_depth_to_color/camera_info'), 
          ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
          
          # OUTPUT: This sends the result directly to PX4
          ('odom', '/mavros/odometry/out') 
    ] 


    return LaunchDescription([

        DeclareLaunchArgument(
            'args', default_value='',
            description='Extra arguments set to rtabmap and odometry nodes.'),
        
        DeclareLaunchArgument(
            'odom_args', default_value='',
            description='Extra args for odometry node.'),

        Node(
            package='rtabmap_odom', 
            executable='rgbd_odometry', 
            output='screen',
            parameters=parameters,
            arguments=[LaunchConfiguration("args"), LaunchConfiguration("odom_args")],
            remappings=remappings
        ),
        
        Node(
            package='midcone_rtabmap',
            executable='vio_bridge',
            output='screen'
        ),
    ])