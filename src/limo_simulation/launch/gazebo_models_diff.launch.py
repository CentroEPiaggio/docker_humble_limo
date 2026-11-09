
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from limo_simulation import launch_utils as lu


def spawn_robots(context):
    n_robots = int(LaunchConfiguration('n_robots').perform(context))
    if n_robots < 1:
        raise ValueError('n_robots must be >= 1')
    seed = LaunchConfiguration('seed').perform(context)
    poses = lu.random_poses(n_robots, seed=int(seed) if seed else None)
    rviz = lu.rviz_action(n_robots, LaunchConfiguration('rvizconfig').perform(context),
                          IfCondition(LaunchConfiguration('rviz')))
    return [rviz] + lu.robots_actions(
        lu.robot_names(n_robots), poses,
        with_arm=lu.as_bool(LaunchConfiguration('with_arm').perform(context)))


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('n_robots', default_value='1',
                              description='Number of robots to spawn'),
        DeclareLaunchArgument('seed', default_value='',
                              description='Random seed for spawn poses (empty = random)'),
        DeclareLaunchArgument('with_arm', default_value='false',
                              description='Mount the myCobot 280 arm + gripper on every robot'),
        DeclareLaunchArgument('gui', default_value='true', description='Start the Gazebo client'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start rviz'),
        DeclareLaunchArgument('rvizconfig', default_value='',
                              description='Absolute path to rviz config file '
                                          '(empty = generated for n_robots)'),
        *lu.gazebo_actions(lu.share_path('world/my_world.world'), LaunchConfiguration('gui')),
        OpaqueFunction(function=spawn_robots),
    ])
