from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lio_sam_ros2',
            executable='lio_sam_ros2_imuPreintegration',
            output='screen'
        ),
        Node(
            package='lio_sam_ros2',
            executable='lio_sam_ros2_imageProjection',
            output='screen'
        ),
        Node(
            package='lio_sam_ros2',
            executable='lio_sam_ros2_featureExtraction',
            output='screen'
        ),
        Node(
            package='lio_sam_ros2',
            executable='lio_sam_ros2_mapOptimization',
            output='screen'
        )
    ])
