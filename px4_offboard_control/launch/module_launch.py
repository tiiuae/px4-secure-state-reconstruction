import launch
import launch_ros.actions

def generate_launch_description():
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
                {"attack": True},
            ],
            name='attacker'),

        launch_ros.actions.Node(
            package='px4_offboard_control',
            executable='offboard_control_xvel',
            output='screen',
            name='offboard_control_xvel'),

        # launch_ros.actions.Node(
        #     package='px4_offboard_control',
        #     executable='offboard_control',
        #     output='screen',
        #     name='offboard_control'),
        #
        # launch_ros.actions.Node(
        #     package='px4_offboard_control',
        #     executable='ekf_2',
        #     output='screen',
        #     name='ekf_2'),
        #
        # launch_ros.actions.Node(
        #     package='px4_offboard_control',
        #     executable='attacker',
        #     output='screen',
        #     parameters=[
        #         {"attack": False},
        #     ],
        #     name='attacker'),

  ])
