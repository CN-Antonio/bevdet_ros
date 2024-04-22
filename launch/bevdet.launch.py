import launch
import launch_ros.actions
# from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = launch.LaunchDescription([
        # bevdet
        launch.actions.DeclareLaunchArgument(
            name='config',
            default_value='config/flashocc.yaml',       # To Modify
            description='bevdet config yaml file'
        ),

        # lauch occdet
        launch_ros.actions.Node(
            package='bevdet',
            namespace='bevdet', 
            executable='bevdet_node',
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