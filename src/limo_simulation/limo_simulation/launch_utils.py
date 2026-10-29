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
CONTROLLER_FILE = 'config/diff_drive_controller.yaml'
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


def controller_config(robot_name):
    """Per-robot copy of the controller yaml with namespaced odometry frames.

    The /** wildcards are replaced by the robot namespace: on Foxy the controller
    manager inside gazebo_ros2_control does not see wildcard parameters (update_rate).
    """
    with open(share_path(CONTROLLER_FILE)) as f:
        config = yaml.safe_load(f)
    config = {key.replace('/**/', f'/{robot_name}/'): value for key, value in config.items()}
    params = config[f'/{robot_name}/diff_drive_base_controller']['ros__parameters']
    params['odom_frame_id'] = f'{robot_name}/odom'
    params['base_frame_id'] = f'{robot_name}/base_footprint'
    path = os.path.join(tmp_dir(), f'{robot_name}_controllers.yaml')
    with open(path, 'w') as f:
        yaml.safe_dump(config, f)
    return path


def robot_description(robot_name, config_path=None):
    mappings = {'robot_name': robot_name, 'namespace': robot_name}
    if config_path:
        mappings['controller_config'] = config_path
    return xacro.process_file(share_path(MODEL_FILE), mappings=mappings).toxml()


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
