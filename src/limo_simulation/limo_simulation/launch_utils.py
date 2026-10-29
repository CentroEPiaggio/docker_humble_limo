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


def robot_description(robot_name, config_path=None):
    mappings = {'robot_name': robot_name, 'namespace': robot_name}
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
