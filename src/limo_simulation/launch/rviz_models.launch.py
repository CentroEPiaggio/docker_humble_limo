from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from limo_simulation import launch_utils as lu


def spawn_robots(context):
    n_robots = int(LaunchConfiguration('n_robots').perform(context))
    if n_robots < 1:
        raise ValueError('n_robots must be >= 1')
    gui = LaunchConfiguration('gui').perform(context).lower() in ('true', '1')
    spacing = float(LaunchConfiguration('spacing').perform(context))

    actions = [
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', lu.rviz_config(n_robots, with_sensors=False)],
        )
    ]

    for robot_name, (x, y, z, yaw) in zip(lu.robot_names(n_robots),
                                          lu.grid_poses(n_robots, spacing)):
        actions += [
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=robot_name,
                output='screen',
                parameters=[{'robot_description': lu.robot_description(robot_name)}],
            ),
            Node(
                package='joint_state_publisher_gui' if gui else 'joint_state_publisher',
                executable='joint_state_publisher_gui' if gui else 'joint_state_publisher',
                namespace=robot_name,
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='world_to_base',
                namespace=robot_name,
                arguments=['--x', str(x), '--y', str(y), '--z', str(z), '--yaw', str(yaw),
                           '--frame-id', 'world',
                           '--child-frame-id', f'{robot_name}/base_footprint'],
            ),
        ]

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('n_robots', default_value='1',
                              description='Number of robots to display'),
        DeclareLaunchArgument('gui', default_value='false',
                              description='Use joint_state_publisher_gui to move the wheels'),
        DeclareLaunchArgument('spacing', default_value='0.5',
                              description='Grid spacing between robots [m]'),
        OpaqueFunction(function=spawn_robots),
    ])
