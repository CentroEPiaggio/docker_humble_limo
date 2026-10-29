from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def teleop(context):
    robot = LaunchConfiguration('robot').perform(context)
    return [
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            namespace=robot,
            # the keyboard node needs its own terminal: launch does not forward stdin
            prefix=LaunchConfiguration('terminal'),
            parameters=[{
                'stamped': True,
                'frame_id': f'{robot}/base_footprint',
                'use_sim_time': True,
            }],
            remappings=[('cmd_vel', 'diff_drive_base_controller/cmd_vel')],
            output='screen',
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='robot_0',
                              description='Name of the robot to drive (robot_0, robot_1, ...)'),
        DeclareLaunchArgument('terminal', default_value='xterm -e',
                              description='Terminal used to read the keyboard'),
        OpaqueFunction(function=teleop),
    ])
