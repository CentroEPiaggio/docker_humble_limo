from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def teleop(context):
    robot = LaunchConfiguration('robot').perform(context)
    return [
        Node(
            package='limo_simulation',
            executable='arm_teleop_keyboard.py',
            name='arm_teleop_keyboard',
            namespace=robot,
            # the keyboard node needs its own terminal: launch does not forward stdin
            prefix=LaunchConfiguration('terminal'),
            parameters=[{'use_sim_time': True}],
            output='screen',
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='robot_0',
                              description='Robot whose arm to move (robot_0, robot_1, ...)'),
        DeclareLaunchArgument('terminal', default_value='xterm -e',
                              description='Terminal used to read the keyboard'),
        OpaqueFunction(function=teleop),
    ])
