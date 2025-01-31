from ament_index_python.packages import get_package_share_directory
import os
import launch
import launch_ros.actions

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('px4_offboard_control'),
        'config',
        'attacker_params.yaml'
        )
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='px4_offboard_control',
            executable='sensor_matrix_downsampler',
            output='screen',
            name='sensor_matrix_downsampler'),

        launch_ros.actions.Node(
            package='px4_offboard_control',
            executable='attacker',
            output='screen',
            parameters=[
                config
            ],
            name='attacker'),

        launch_ros.actions.Node(
            package='px4_offboard_control',
            executable='offboard_control_xvel',
            output='screen',
            name='offboard_control_xvel'),
  ])
