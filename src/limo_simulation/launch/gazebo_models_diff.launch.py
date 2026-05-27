import os
import sys
import xacro
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from scripts import GazeboRosPaths
import random


def generate_launch_description():
    n_robots = 1

    for arg in sys.argv:
        if arg.startswith("n_robots:="):
            n_robots = int(arg.split(":=")[1])
    
    P = [
        [
            random.uniform(-2.0, 2.0),   # x
            random.uniform(-2.0, 2.0),   # y
            0,                            # z
            random.uniform(-3.14, 3.14)  # yaw
        ]
        for _ in range(n_robots)
    ]
    

    # Constants for paths to different files and folders
    name_package = 'limo_simulation'
    modelFileRelativePath = 'model/limo_four_diff.xacro'
    worldFileRelativePath = 'world/my_world.world'

    pathModelFile = os.path.join(get_package_share_path(name_package), modelFileRelativePath)
    pathWorldFile = os.path.join(get_package_share_path(name_package), worldFileRelativePath)
    # robotDescription = xacro.process_file(pathModelFile).toxml()


    gazebo_rosPakageLaunch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_path('gazebo_ros'), 'launch', 'gazebo.launch.py')
    )
    gazeboLaunch = IncludeLaunchDescription(
        gazebo_rosPakageLaunch,
        launch_arguments={'world': pathWorldFile}.items(),
    )

    spawnRobots = []
    robotsStatePub = []
    robotsStateBrod = []
    robotsControllers = []

    # publishers = []
    for i in range(n_robots):
        robot_name = f'robot_{i}'
        robotDescription = xacro.process_file(
            pathModelFile, mappings={'robot_name': robot_name, 'namespace': robot_name}
        ).toxml()
        publisher_name = f'robot_state_publisher'

        # Node to publish the state of the robot to tf
        robotStatePubNode = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name=publisher_name,
            namespace=robot_name,
            output='screen',
            parameters=[{'robot_description': robotDescription, 'use_sim_time': True}],
        )

        robotsStatePub.append(robotStatePubNode)
        spawner_name = f'spawn_entity_{robot_name}'
        # Node to spawn the robot in gazebo
        spawnModelNode = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name=spawner_name,
            arguments=[
                '-topic',
                f'/{robot_name}/robot_description',
                '-robot_namespace',
                robot_name,
                '-entity',
                f'/{robot_name}',
                '-x',
                str(P[i][0]),
                '-y',
                str(P[i][1]),
                '-z',
                str(P[i][2]),
                '-Y',
                str(P[i][3]),
            ],
            namespace=robot_name,
            output='screen',
        )
        spawnRobots.append(spawnModelNode)

        broadcaster_namespace = f'joint_state_broadcaster_{i}'

        controller_namespace = f'diff_drive_base_controller_{i}'

        load_joint_state_broadcaster = Node(
            package='controller_manager',
            executable='spawner',
            name=broadcaster_namespace,
            namespace=robot_name,
            arguments=[
                'joint_state_broadcaster',
                '--controller-manager',
                f'/{robot_name}/controller_manager',
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        )
        robotsStateBrod.append(load_joint_state_broadcaster)

        load_diff_drive_base_controller = Node(
            package='controller_manager',
            executable='spawner',
            name=controller_namespace,
            namespace=robot_name,
            arguments=[
                'diff_drive_base_controller',
                '--controller-manager',
                f'/{robot_name}/controller_manager',
            ],
            parameters=[{'use_sim_time': True}],
            output='screen',
        )
        robotsControllers.append(load_diff_drive_base_controller)

    LaunchDescriptionObject = LaunchDescription()
    LaunchDescriptionObject.add_action(gazeboLaunch)

    for i in range(n_robots):
        LaunchDescriptionObject.add_action(spawnRobots[i])
        LaunchDescriptionObject.add_action(robotsStatePub[i])
        LaunchDescriptionObject.add_action(robotsStateBrod[i])
        LaunchDescriptionObject.add_action(robotsControllers[i])

    return LaunchDescriptionObject
