import launch
import launch_ros.actions
# from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = launch.LaunchDescription([
        # TODO: sub topics
        launch.actions.DeclareLaunchArgument(
            name='fl_cam_topic',
            default_value='/CAM_FRONT_LEFT/image_raw',
            description='front left camera topic'
        ),

        # TODO: pub topic (MarkerArray)

        # bevdet
        launch.actions.DeclareLaunchArgument(
            name='config',
            default_value='config/flashocc.yaml',       # To Modify
            description='bevdet config yaml file'
        ),

        # lauch occdet
        launch_ros.actions.Node(
            package='bevdet_ros',
            namespace='bevdet_ros', 
            executable='bevdet_ros_node',
            name='bevdet_node_1',
            parameters=[{
                'config': launch.substitutions.LaunchConfiguration('config'),
            }],
            output='screen'
        )
    ])
    return ld

if __name__ == '__main__':
    generate_launch_description()