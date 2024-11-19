
import os

from ament_index_python import get_package_share_directory

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetLaunchConfiguration
from launch.actions import GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():

    remappings=[
        ('/camera/camera/aligned_depth_to_color/image_raw', '/camera/aligned_depth_to_color/image_raw'),
        ('/camera/camera/aligned_depth_to_color/camera_info', '/camera/aligned_depth_to_color/camera_info'),
        ('/camera/camera/aligned_depth_to_color/image_raw/compressed', '/camera/aligned_depth_to_color/image_raw/compressed'),
        ('/camera/camera/aligned_depth_to_color/image_raw/compressedDepth', '/camera/aligned_depth_to_color/image_raw/compressedDepth'),
        ('/camera/camera/aligned_depth_to_color/image_raw/theora', '/camera/aligned_depth_to_color/image_raw/theora'),
        ('/camera/camera/accel/imu_info', '/camera/accel/imu_info'),
        ('/camera/camera/accel/metadata', '/camera/accel/metadata'),
        ('/camera/camera/accel/sample', '/camera/accel/sample'),
        ('/camera/camera/color/image_raw/compressed', '/camera/color/image_raw/compressed'),
        ('/camera/camera/color/image_raw/compressedDepth', '/camera/color/image_raw/compressedDepth'),
        ('/camera/camera/color/image_raw/theora', '/camera/color/image_raw/theora'),
        ('/camera/camera/color/metadata', '/camera/color/metadata'),
        ('/camera/camera/depth/camera_info', '/camera/depth/camera_info'),
        ('/camera/camera/depth/image_rect_raw', '/camera/depth/image_rect_raw'),
        ('/camera/camera/depth/image_rect_raw/compressed', '/camera/depth/image_rect_raw/compressed'),
        ('/camera/camera/depth/image_rect_raw/compressedDepth', '/camera/depth/image_rect_raw/compressedDepth'),
        ('/camera/camera/depth/image_rect_raw/theora', '/camera/depth/image_rect_raw/theora'),
        ('/camera/camera/depth/metadata', '/camera/depth/metadata'),
        ('/camera/camera/extrinsics/depth_to_accel', '/camera/extrinsics/depth_to_accel'),
        ('/camera/camera/extrinsics/depth_to_color', '/camera/extrinsics/depth_to_color'),
        ('/camera/camera/extrinsics/depth_to_gyro', '/camera/extrinsics/depth_to_gyro'),
        ('/camera/camera/gyro/imu_info', '/camera/gyro/imu_info'),
        ('/camera/camera/gyro/metadata', '/camera/gyro/metadata'),
        ('/camera/camera/gyro/sample', '/camera/gyro/sample'),
        ('/camera/camera/imu', '/camera/imu'),
        ('/camera/camera/color/camera_info', '/camera/color/camera_info'),
        ('/camera/camera/color/image_raw', '/camera/color/image_raw')  
    ]

    enable_accel_arg = DeclareLaunchArgument(
        "enable_accel",
        default_value=TextSubstitution(text="false")
    )
    enable_gyro_arg = DeclareLaunchArgument(
        "enable_gyro",
        default_value=TextSubstitution(text="false")
    )

    align_depth_arg = DeclareLaunchArgument(
        "align_depth.enable", 
        default_value=TextSubstitution(text="true")
    )

    linear_accel_cov_arg = DeclareLaunchArgument(
        "linear_accel_cov", 
        default_value=TextSubstitution(text="1.0")
    )

    unite_imu_method_arg = DeclareLaunchArgument(
        "unite_imu_method",
        default_value=TextSubstitution(text="2")
    )

    initial_reset_arg = DeclareLaunchArgument(
        "initial_reset",
        default_value=TextSubstitution(text="true")
    )

    rs2_camera_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(           
             os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )),
        launch_arguments={
            'remappings': str(remappings),
            'enable_accel': LaunchConfiguration('enable_accel'),
            'enable_gyro': LaunchConfiguration('enable_gyro'),
            'align_depth.enable': LaunchConfiguration('align_depth.enable'), 
            'linear_accel_cov': LaunchConfiguration('linear_accel_cov'),
            'unite_imu_method': LaunchConfiguration('unite_imu_method'),
            'initial_reset': LaunchConfiguration('initial_reset')}.items(),
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_host',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('slam_toolbox'), 'config', 'slam_toolbox_d435i.rviz')]
    )



    return LaunchDescription([

        rviz2_node,        
        enable_accel_arg,
        enable_gyro_arg,
        align_depth_arg,    
        linear_accel_cov_arg,
        unite_imu_method_arg,
        initial_reset_arg,
        rs2_camera_launch_include      
    ])