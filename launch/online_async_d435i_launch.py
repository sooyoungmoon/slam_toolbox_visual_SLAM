
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
        default_value=TextSubstitution(text="true")
    )
    enable_gyro_arg = DeclareLaunchArgument(
        "enable_gyro",
        default_value=TextSubstitution(text="true")
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
            'unite_imu_method': LaunchConfiguration('unite_imu_method')}.items()            
                 
    )

    depthimage_to_laserscan_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('depthimage_to_laserscan'),
                'launch',
                'depthimage_to_laserscan-launch.py'
            )
        )
    )

    imu_filter_madgwick_node = Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='ImuFilter',
            remappings=[
                ('/imu/data_raw', '/camera/imu')
            ],
            parameters=[{
                'use_mag': False,
                'publish_tf': False,
                'world_frame': 'enu',
            }]
        )

    rtabmap_rgbd_odometry_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',

        parameters=[{
            'frame_id': 'camera_link',
            'odom_frame_id': 'odom',
            'publish_tf': True,

        }],
        remappings=[
            ("rgb/image", "/camera/color/image_raw"),
            ("depth/image", "/camera/depth/image_rect_raw"),
            ("rgb/camera_info", "/camera/color/camera_info")
        ]
    )

    # rtabmap_launch_include = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(
    #             os.path.join(
    #                 get_package_share_directory('rtabmap_launch'),
    #                 'launch/rtabmap.launch.py'))     
    #     )   

    slam_toolbox_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch/online_async_launch.py'))     
    )   

    my_tf2_publisher_node = Node(
        package='imu_to_odom',
        executable='myTfBroacaster',
        name='myTfBroacaster',
        output='screen'
        
    )

    tf2_publisher_node_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments= ["0", "0", "0", "0", "0", "0", "map", "odom"]
    )

    tf2_publisher_node_2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments= ["0", "0", "0", "0", "0", "0", "base_link", "camera_link"]
    )

    tf2_publisher_node_3 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments= ["0", "0", "0", "0", "0", "0", "odom", "base_footprint"]
    )

    robot_localization_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_localization'),
                'launch/ukf.launch.py'))     
    )   


    return LaunchDescription([
        #
        depthimage_to_laserscan_include,
        enable_accel_arg,
        enable_gyro_arg,
        align_depth_arg,    
        linear_accel_cov_arg,
        unite_imu_method_arg,
        imu_filter_madgwick_node,
        rs2_camera_launch_include,
        my_tf2_publisher_node,
        
        tf2_publisher_node_1,
        #tf2_publisher_node_2,
        #tf2_publisher_node_3,
        slam_toolbox_launch_include,
        #robot_localization_launch_include,
        rtabmap_rgbd_odometry_node
    ])
