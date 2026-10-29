import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from limo_simulation import launch_utils as lu


def spawn_robots(context):
    n_robots = int(LaunchConfiguration('n_robots').perform(context))
    if n_robots < 1:
        raise ValueError('n_robots must be >= 1')
    seed = LaunchConfiguration('seed').perform(context)
    poses = lu.random_poses(n_robots, seed=int(seed) if seed else None)

    rviz_config = LaunchConfiguration('rvizconfig').perform(context)
    if not rviz_config:
        rviz_config = lu.rviz_config(n_robots, with_sensors=True)

    actions = [
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        )
    ]

    previous_step = None
    for robot_name, (x, y, z, yaw) in zip(lu.robot_names(n_robots), poses):
        robot_description = lu.robot_description(robot_name)

        # Node to publish the state of the robot to tf
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=robot_name,
            output='screen',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        )

        # Wheel odometry starts at the spawn pose: anchor each odom frame in the world frame.
        # base_footprint is attached to world by the ground truth, so robot_N/odom is only
        # the reference frame of /robot_N/diff_drive_base_controller/odom
        world_to_odom = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_odom',
            namespace=robot_name,
            arguments=['--x', str(x), '--y', str(y), '--yaw', str(yaw),
                       '--frame-id', 'world', '--child-frame-id', f'{robot_name}/odom'],
            parameters=[{'use_sim_time': True}],
        )

        # Sensors and ground truth (world -> base_footprint tf) from Gazebo to ROS
        bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_bridge',
            namespace=robot_name,
            parameters=[{'config_file': lu.bridge_config(robot_name), 'use_sim_time': True}],
            output='screen',
        )

        # Node to spawn the robot in gazebo
        spawn_entity = Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_entity',
            namespace=robot_name,
            arguments=[
                '-topic', f'/{robot_name}/robot_description',
                '-name', robot_name,
                '-x', str(x), '-y', str(y), '-z', str(z), '-Y', str(yaw),
            ],
            output='screen',
        )

        def spawner(controller, robot_name=robot_name):
            return Node(
                package='controller_manager',
                executable='spawner',
                namespace=robot_name,
                arguments=[controller, '--controller-manager', f'/{robot_name}/controller_manager'],
                output='screen',
            )

        joint_state_broadcaster = spawner('joint_state_broadcaster')
        diff_drive_controller = spawner('diff_drive_base_controller')

        actions += [robot_state_publisher, world_to_odom, bridge]
        # the ros2_control Gazebo plugin stores its namespace and params file in the
        # process-global ROS arguments, so controllers loaded after the next robot is spawned would end up in
        # that robot's namespace. Spawn robot by robot: entity -> broadcaster -> diff drive.
        if previous_step is None:
            actions.append(spawn_entity)
        else:
            actions.append(RegisterEventHandler(OnProcessExit(
                target_action=previous_step, on_exit=[spawn_entity])))
        actions += [
            RegisterEventHandler(OnProcessExit(
                target_action=spawn_entity, on_exit=[joint_state_broadcaster])),
            RegisterEventHandler(OnProcessExit(
                target_action=joint_state_broadcaster, on_exit=[diff_drive_controller])),
        ]
        previous_step = diff_drive_controller

    return actions


def generate_launch_description():
    world_file = lu.share_path('world/my_world.world')

    # -r: start running, -s: server only (no GUI)
    gz_args = PythonExpression([
        "'-r ", world_file, "' + ('' if '", LaunchConfiguration('gui'),
        "'.lower() in ('true', '1') else ' -s')"])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_path('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('n_robots', default_value='1',
                              description='Number of robots to spawn'),
        DeclareLaunchArgument('seed', default_value='',
                              description='Random seed for spawn poses (empty = random)'),
        DeclareLaunchArgument('gui', default_value='true', description='Start the Gazebo client'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start rviz'),
        DeclareLaunchArgument('rvizconfig', default_value='',
                              description='Absolute path to rviz config file '
                                          '(empty = generated for n_robots)'),
        gazebo,
        clock_bridge,
        OpaqueFunction(function=spawn_robots),
    ])
