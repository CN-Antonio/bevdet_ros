import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = launch.LaunchDescription([
        # TODO: sub topics
        launch.actions.DeclareLaunchArgument(
            name='cam_fl_topic',
            # default_value='/CAM_FRONT_LEFT/image_raw',
            default_value='/carla/hero/cam_front_left/image',
            description='front left camera topic'
        ),
        launch.actions.DeclareLaunchArgument(
            name='cam_f_topic',
            # default_value='/CAM_FRONT/image_raw',
            default_value='/carla/hero/cam_front/image',
            description='front left camera topic'
        ),
        launch.actions.DeclareLaunchArgument(
            name='cam_fr_topic',
            # default_value='/CAM_FRONT_RIGHT/image_raw',
            default_value='/carla/hero/cam_front_right/image',
            description='front left camera topic'
        ),
        launch.actions.DeclareLaunchArgument(
            name='cam_bl_topic',
            # default_value='/CAM_BACK_LEFT/image_raw',
            default_value='/carla/hero/cam_back_left/image',
            description='front left camera topic'
        ),
        launch.actions.DeclareLaunchArgument(
            name='cam_b_topic',
            # default_value='/CAM_BACK/image_raw',
            default_value='/carla/hero/cam_back/image',
            description='front left camera topic'
        ),
        launch.actions.DeclareLaunchArgument(
            name='cam_br_topic',
            # default_value='/CAM_BACK_RIGHT/image_raw',
            default_value='/carla/hero/cam_back_right/image',
            description='front left camera topic'
        ),

        # TODO: pub topic (MarkerArray)

        # bevdet
        launch.actions.DeclareLaunchArgument(
            name='configure',
            default_value=os.path.join(
                get_package_share_directory('bevdet_ros'),
                'config/bevdet/configure.yaml'),
            description='bevdet config yaml file'
        ),
        launch.actions.DeclareLaunchArgument(
            name='ModelConfig',
            default_value=os.path.join(
                get_package_share_directory('bevdet_ros'),
                'config/bevdet/cfgs/bevdet_lt_depth.yaml'),
            description='model config yaml file'
        ),
        launch.actions.DeclareLaunchArgument(
            name='imgstage',
            default_value=os.path.join(
                get_package_share_directory('bevdet_ros'),
                'model/img_stage_lt_d_fp16.engine'),
            description='imgstage engine file'
        ),
        launch.actions.DeclareLaunchArgument(
            name='bevstage',
            default_value=os.path.join(
                get_package_share_directory('bevdet_ros'),
                'model/bev_stage_lt_d_fp16.engine'),
            description='bevstage engine file'
        ),

        # lauch occdet
        launch_ros.actions.Node(
            package='bevdet_ros',
            namespace='bevdet_ros', 
            executable='bevdet_ros_node',
            name='bevdet_node_1',
            parameters=[{
                'cam_fl_topic': launch.substitutions.LaunchConfiguration('cam_fl_topic'),
                'cam_f_topic': launch.substitutions.LaunchConfiguration('cam_f_topic'),
                'cam_fr_topic': launch.substitutions.LaunchConfiguration('cam_fr_topic'),
                'cam_bl_topic': launch.substitutions.LaunchConfiguration('cam_bl_topic'),
                'cam_b_topic': launch.substitutions.LaunchConfiguration('cam_b_topic'),
                'cam_br_topic': launch.substitutions.LaunchConfiguration('cam_br_topic'),
                'configure': launch.substitutions.LaunchConfiguration('configure'),
                'model': launch.substitutions.LaunchConfiguration('ModelConfig'),
                'imgstage': launch.substitutions.LaunchConfiguration('imgstage'),
                'bevstage': launch.substitutions.LaunchConfiguration('bevstage'),
            }],
            remappings=[
                # ('/carla/hero/cam_front_left/image', '/CAM_FRONT_LEFT/image_raw'),
                # ('/carla/hero/cam_front/image', '/CAM_FRONT/image_raw'),
                # ('/carla/hero/cam_front_right/image', '/CAM_FRONT_RIGHT/image_raw'),
                # ('/carla/hero/cam_back_left', '/CAM_BACK_LEFT/image_raw'),
                # ('/carla/hero/cam_back', '/CAM_BACK/image_raw'),
                # ('/carla/hero/cam_back_right', '/CAM_BACK_RIGHT/image_raw'),
            ],
            output='screen'
        )
    ])
    return ld

if __name__ == '__main__':
    generate_launch_description()