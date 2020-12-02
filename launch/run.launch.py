import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    mapping_param_dir = launch.substitutions.LaunchConfiguration(
            'mapping_param_dir',
            default=os.path.join(
                get_package_share_directory('lio_sam_ros2'), 'config', 'params.yaml'
                )
            )

    image_projection = launch_ros.actions.Node(
            package='lio_sam_ros2', 
            executable='lio_sam_ros2_imageProjection',
            parameters=[mapping_param_dir],
            output='screen'
            )
    '''
    imu_preintegration = launch_ros.actions.Node(
            package='lio_sam_ros2',
            executable='lio_sam_ros2_imuPreintegration',
            parameters=[mapping_param_dir],
            output='screen'
            )

    feature_extraction = launch_ros.actions.Node(
            package='lio_sam_ros2',
            executable='lio_sam_ros2_featureExtraction',
            parameters=[mapping_param_dir],
            output='screen'
            )

    map_optimization = launch_ros.actions.Node(
            package='lio_sam_ros2',
            executable='lio_sam_ros2_mapOptimization',
            parameters=[mapping_param_dir],
            output='screen'
            )
    '''
    return launch.LaunchDescription(
            [
            launch.actions.DeclareLaunchArgument(
                'mapping_param_dir',
                default_value=mapping_param_dir
                ),
            image_projection,
            #imu_preintegration,
            #feature_extraction,
            #map_optimization
            ]
            )
