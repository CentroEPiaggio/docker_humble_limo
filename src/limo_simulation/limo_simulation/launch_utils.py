"""Helpers shared by the multi-robot launch files."""

import copy
import math
import os
import random
import tempfile

import xacro
import yaml
from ament_index_python.packages import get_package_share_path

PACKAGE = 'limo_simulation'
MODEL_FILE = 'model/limo_four_diff.xacro'
ARM_CONTROLLER_CONFIG = 'config/limo_mycobot_controllers.yaml'
RVIZ_TEMPLATE = 'rviz/limo_display.rviz'
# drop the robot from slightly above the ground for a clean first contact
SPAWN_Z = 0.01

_tmp_dir = None


def share_path(relative_path):
    return os.path.join(get_package_share_path(PACKAGE), relative_path)


def tmp_dir():
    global _tmp_dir
    if _tmp_dir is None:
        _tmp_dir = tempfile.mkdtemp(prefix='limo_sim_')
    return _tmp_dir


def robot_names(n_robots):
    return [f'robot_{i}' for i in range(n_robots)]


def robot_description(robot_name, config_path=None, with_arm=False):
    """with_arm: mount the myCobot 280 and load its controllers (unless config_path is given)."""
    mappings = {'robot_name': robot_name, 'namespace': robot_name,
                'with_arm': 'true' if with_arm else 'false'}
    if with_arm and not config_path:
        config_path = share_path(ARM_CONTROLLER_CONFIG)
    if config_path:
        mappings['controller_config'] = config_path
    return xacro.process_file(share_path(MODEL_FILE), mappings=mappings).toxml()


# (gz topic suffix, ros topic suffix, ros type, gz type) bridged for every robot, Gazebo -> ROS
_ROBOT_BRIDGES = [
    ('scan', 'scan', 'sensor_msgs/msg/LaserScan', 'ignition.msgs.LaserScan'),
    ('imu', 'imu', 'sensor_msgs/msg/Imu', 'ignition.msgs.IMU'),
    ('camera_ir/image', 'camera_ir/image_raw', 'sensor_msgs/msg/Image', 'ignition.msgs.Image'),
    ('camera_ir/camera_info', 'camera_ir/camera_info', 'sensor_msgs/msg/CameraInfo',
     'ignition.msgs.CameraInfo'),
    ('camera_ir/depth_image', 'camera_ir/depth/image_raw', 'sensor_msgs/msg/Image',
     'ignition.msgs.Image'),
    ('camera_ir/points', 'camera_ir/depth/points', 'sensor_msgs/msg/PointCloud2',
     'ignition.msgs.PointCloudPacked'),
]


def bridge_config(robot_name):
    """Write a ros_gz_bridge config for one robot's sensors and ground truth; return its path."""
    entries = [{
        'ros_topic_name': f'/{robot_name}/{ros}', 'gz_topic_name': f'/{robot_name}/{gz}',
        'ros_type_name': ros_type, 'gz_type_name': gz_type, 'direction': 'GZ_TO_ROS',
    } for gz, ros, ros_type, gz_type in _ROBOT_BRIDGES]
    entries += [
        # ground truth world -> base_footprint, see OdometryPublisher in the model
        {'ros_topic_name': '/tf', 'gz_topic_name': f'/model/{robot_name}/tf',
         'ros_type_name': 'tf2_msgs/msg/TFMessage', 'gz_type_name': 'ignition.msgs.Pose_V',
         'direction': 'GZ_TO_ROS'},
        {'ros_topic_name': f'/{robot_name}/ground_truth/odom',
         'gz_topic_name': f'/model/{robot_name}/odometry',
         'ros_type_name': 'nav_msgs/msg/Odometry', 'gz_type_name': 'ignition.msgs.Odometry',
         'direction': 'GZ_TO_ROS'},
    ]
    path = os.path.join(tmp_dir(), f'{robot_name}_bridge.yaml')
    with open(path, 'w') as f:
        yaml.safe_dump(entries, f)
    return path


def random_poses(n, half_size=2.0, min_dist=0.5, seed=None, max_tries=10000):
    """Random [x, y, z, yaw] poses inside a square, at least min_dist apart."""
    rng = random.Random(seed)
    poses = []
    tries = 0
    while len(poses) < n:
        tries += 1
        if tries > max_tries:
            raise RuntimeError(
                f'Cannot place {n} robots {min_dist} m apart in a '
                f'{2 * half_size} m square; reduce n_robots')
        x = rng.uniform(-half_size, half_size)
        y = rng.uniform(-half_size, half_size)
        if all(math.hypot(x - p[0], y - p[1]) >= min_dist for p in poses):
            poses.append([x, y, SPAWN_Z, rng.uniform(-math.pi, math.pi)])
    return poses


def grid_poses(n, spacing=0.5):
    """[x, y, z, yaw] poses on a square grid centred on the origin."""
    cols = math.ceil(math.sqrt(n))
    offset = (cols - 1) * spacing / 2.0
    return [[(i % cols) * spacing - offset, (i // cols) * spacing - offset, 0.0, 0.0]
            for i in range(n)]


def rviz_config(n_robots, with_sensors):
    """Build an rviz config with one RobotModel (and optionally sensors) per robot."""
    with open(share_path(RVIZ_TEMPLATE)) as f:
        config = yaml.safe_load(f)
    displays = config['Visualization Manager']['Displays']
    templates = {d['Class']: d for d in displays}
    new_displays = [d for d in displays if d['Class'] == 'rviz_default_plugins/Grid']
    new_displays.append({
        'Class': 'rviz_default_plugins/TF', 'Name': 'TF', 'Enabled': False,
        'Value': False, 'Show Names': False,
    })

    topic_classes = {'rviz_default_plugins/RobotModel': 'robot_description'}
    if with_sensors:
        topic_classes['rviz_default_plugins/LaserScan'] = 'scan'
        topic_classes['rviz_default_plugins/Image'] = 'camera_ir/image_raw'

    for name in robot_names(n_robots):
        for cls, topic in topic_classes.items():
            if cls not in templates:
                continue
            d = copy.deepcopy(templates[cls])
            d['Name'] = f'{name} {cls.split("/")[-1]}'
            key = 'Description Topic' if cls.endswith('RobotModel') else 'Topic'
            d[key]['Value'] = f'/{name}/{topic}'
            if cls.endswith('RobotModel'):
                # robot_state_publisher latches the description
                d[key]['Durability Policy'] = 'Transient Local'
            if cls.endswith('Image'):
                # one image panel per robot clutters the window: keep only robot_0
                d['Enabled'] = d['Value'] = name == 'robot_0'
            new_displays.append(d)

    config['Visualization Manager']['Displays'] = new_displays
    config['Visualization Manager']['Global Options']['Fixed Frame'] = 'world'
    path = os.path.join(tmp_dir(), 'multi_robot.rviz')
    with open(path, 'w') as f:
        yaml.safe_dump(config, f)
    return path


# Top-down GUI camera per world: (x, y, z, yaw). Unknown worlds get the default.
TOP_VIEWS = {}
DEFAULT_TOP_VIEW = (0.0, 0.0, 15.0, math.pi / 2)


def top_view_action(world_file, condition=None):
    """Move the Gazebo GUI camera above the world, looking down (retries until the GUI is up)."""
    from launch.actions import ExecuteProcess
    x, y, z, yaw = TOP_VIEWS.get(os.path.basename(world_file), DEFAULT_TOP_VIEW)
    # rpy (0, pi/2, yaw) -> quaternion: pitch pi/2 looks straight down
    s = c = math.sqrt(0.5)
    qw, qx = c * math.cos(yaw / 2), -s * math.sin(yaw / 2)
    qy, qz = s * math.cos(yaw / 2), c * math.sin(yaw / 2)
    req = (f'pose: {{position: {{x: {x}, y: {y}, z: {z}}}, '
           f'orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}}}')
    cmd = ('for i in $(seq 60); do '
           'ign service -s /gui/move_to/pose --reqtype ignition.msgs.GUICamera '
           f'--reptype ignition.msgs.Boolean --timeout 1000 --req "{req}" '
           '| grep -q "data: true" && exit 0; sleep 1; done; '
           'echo "top view: Gazebo GUI camera service not available"')
    return ExecuteProcess(cmd=['bash', '-c', cmd], name='gz_top_view', output='screen',
                         condition=condition)


def gazebo_actions(world_file, gui):
    """Gazebo Fortress with world_file plus the /clock bridge. gui: launch substitution or str."""
    from launch.actions import IncludeLaunchDescription
    from launch.conditions import IfCondition
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.substitutions import PythonExpression
    from launch_ros.actions import Node

    # -r: start running, -s: server only (no GUI)
    gz_args = PythonExpression([
        "'-r ", world_file, "' + ('' if '", gui, "'.lower() in ('true', '1') else ' -s')"])
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
    top_view = top_view_action(world_file, IfCondition(gui))
    return [gazebo, clock_bridge, top_view]


def as_bool(value):
    return str(value).strip().lower() in ('true', '1', 'yes')


def robots_actions(names, poses, with_arm=False):
    """Per-robot nodes (state publisher, odom anchor, bridge, spawn, controllers).

    The ros2_control Gazebo plugin stores its namespace and params file in the process-global
    ROS arguments, so controllers loaded after the next robot is spawned would end up in that
    robot's namespace. Robots are spawned one by one: entity -> broadcaster -> diff drive
    (-> arm + gripper controllers when with_arm).
    """
    from launch.actions import RegisterEventHandler
    from launch.event_handlers import OnProcessExit
    from launch_ros.actions import Node

    actions = []
    previous_step = None
    for robot_name, (x, y, z, yaw) in zip(names, poses):
        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=robot_name,
            output='screen',
            parameters=[{'robot_description': robot_description(robot_name, with_arm=with_arm),
                         'use_sim_time': True}],
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
            parameters=[{'config_file': bridge_config(robot_name), 'use_sim_time': True}],
            output='screen',
        )
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

        def spawner(*controllers, robot_name=robot_name):
            return Node(
                package='controller_manager',
                executable='spawner',
                namespace=robot_name,
                arguments=[*controllers, '--controller-manager',
                           f'/{robot_name}/controller_manager'],
                output='screen',
            )

        joint_state_broadcaster = spawner('joint_state_broadcaster')
        diff_drive_controller = spawner('diff_drive_base_controller')

        actions += [robot_state_publisher, world_to_odom, bridge]
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
        if with_arm:
            arm_controllers = spawner('arm_controller', 'gripper_group_controller')
            # real-robot topic API (mycobot/angles_goal, coords_goal, gripper_status, ...)
            arm_bridge = Node(
                package=PACKAGE,
                executable='mycobot_sim_bridge.py',
                name='mycobot_sim_bridge',
                namespace=robot_name,
                parameters=[{'use_sim_time': True}],
                output='screen',
            )
            actions += [arm_bridge, RegisterEventHandler(OnProcessExit(
                target_action=diff_drive_controller, on_exit=[arm_controllers]))]
            previous_step = arm_controllers
    return actions


def rviz_action(n_robots, rviz_config_path, condition):
    from launch_ros.actions import Node
    return Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path or rviz_config(n_robots, with_sensors=True)],
        parameters=[{'use_sim_time': True}],
        condition=condition,
    )
