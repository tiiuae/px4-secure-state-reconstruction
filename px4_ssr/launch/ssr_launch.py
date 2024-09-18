import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='px4_ssr',
            executable='state_estimator',
            output='screen',
            name='state_estimator'),

        launch_ros.actions.Node(
            package='px4_ssr',
            executable='safe_controller',
            output='screen',
            name='safe_controller'),
  ])

