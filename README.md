# ROS2 Humble with limo repo for simulation installed
## Overview

Docker container for the [limo simulation environment](https://github.com/agilexrobotics/limo_ros2/tree/humble) repo.

The `build.bash` and the `run.bash` files are used to automatically build and run the image.


## Preliminaries

Install [Docker Community Edition](https://docs.docker.com/engine/install/ubuntu/) (ex Docker Engine).
You can follow the installation method through `apt`.
Note that it makes you verify the installation by running `sudo docker run hello-world`.
It is better to avoid running this command with `sudo` and instead follow the post installation steps first and then run the command without `sudo`.

Follow with the [post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/) for Linux.
This will allow you to run Docker without `sudo`.


## Usage

Build the docker image with
```shell
./build.bash [-a] [-f] [-h] [-r]
```
Where the optional arguments represent:
- `-a` or `--all`: build the image with all the dependencies
- `-f` or `--ffmpeg`: build the image with ffmpeg (for saving videos)
- `-h` or `--help`: show the help message
- `-r` or `--rebuild`: rebuild the image
- `-t` or `--torch`: build the image with PyTorch

Run the container with
```shell
./run.bash
```

This repo also supports VS Code devcontainers.


## Simulation

Build the workspace inside the container:
```shell
colcon build && source install/setup.bash
```

### Gazebo with N robots
The simulator is Gazebo Fortress (`ros_gz`, `ign_ros2_control`), the version paired with Humble.
```shell
ros2 launch limo_simulation gazebo_models_diff.launch.py n_robots:=3
```
Arguments:
- `n_robots` (default `1`): number of robots, named `robot_0`, `robot_1`, ...
- `seed` (default random): seed for the spawn poses (random in a 4x4 m square, at least 0.5 m apart)
- `gui` (default `true`): start the Gazebo client
- `rviz` (default `true`): start RViz with a config generated for `n_robots`
- `rvizconfig`: use a custom RViz config instead

Robots are spawned one after the other (entity -> `joint_state_broadcaster` -> `diff_drive_base_controller`), so startup takes a few seconds per robot.

Per-robot interface (`robot_N`):
- command: `/robot_N/diff_drive_base_controller/cmd_vel` (`geometry_msgs/TwistStamped`)
- ground truth: `/robot_N/ground_truth/odom` (pose from Gazebo, in the `world` frame)
- wheel odometry: `/robot_N/diff_drive_base_controller/odom` (estimate in `robot_N/odom`, anchored at the spawn pose; drifts)
- sensors: `/robot_N/scan`, `/robot_N/imu`, `/robot_N/camera_ir/image_raw`, `/robot_N/camera_ir/depth/...`
- TF: `world -> robot_N/base_footprint -> robot_N/...` from the ground truth, so RViz matches Gazebo; `world -> robot_N/odom` is static at the spawn pose

The robot is a four-wheel skid-steer: wheel odometry over-estimates rotation when turning, so it is published only on its topic (no TF). Tune `wheel_separation_multiplier` in `config/diff_drive_controller.yaml` to reduce the drift.

### RViz only (no Gazebo)
```shell
ros2 launch limo_simulation rviz_models.launch.py n_robots:=4 gui:=true
```
Robots are placed on a grid (`spacing`, default 0.5 m); `gui:=true` opens `joint_state_publisher_gui` sliders.

### Keyboard teleoperation
With the simulation running, in another terminal:
```shell
ros2 launch limo_simulation teleop.launch.py robot:=robot_1
```
The keyboard is read in a new `xterm` window (change it with `terminal:='gnome-terminal --'`).

### Test publishers
```shell
ros2 run limo_simulation diff_drive_publisher.py --ros-args -p robot:=robot_0
ros2 run limo_simulation diff_drive_publisher --ros-args -p robot:=robot_0
```

### LIMO + myCobot 280 arm
Every robot can carry the myCobot 280 M5 with the adaptive gripper, mounted as on the real robot (`base_link -> g_base`, yaw +90°):
```shell
ros2 launch limo_simulation gazebo_models_diff.launch.py n_robots:=2 with_arm:=true
```
`with_arm` defaults to `false`, so existing launches are unchanged.

Packages:
- `mycobot_description`: arm + gripper xacro (same joint names and kinematics as the real `mycobot_280_m5_adaptive_gripper.urdf`, with inertials and Gazebo mesh paths) and meshes
- `mycobot_interfaces`: the real robot messages/services (`MycobotSetAngles`, `MycobotSetCoords`, `MycobotGripperStatus`, ...)

The arm joints share the robot's single `ros2_control` block (`config/limo_mycobot_controllers.yaml`):
- `arm_controller` (`JointTrajectoryController`): 6 arm joints, `/robot_N/arm_controller/joint_trajectory` or the `follow_joint_trajectory` action
- `gripper_group_controller` (`JointGroupPositionController`): gripper driver + its 5 linkage joints (the former `<mimic>` joints)

`mycobot_sim_bridge.py` runs in every robot namespace and exposes the real robot topic API (`mycobot_topics.py`), so scripts written for the real arm work in simulation:

| Topic (`/robot_N/...`) | Type | Notes |
|---|---|---|
| `mycobot/angles_goal` | `MycobotSetAngles` | degrees, `speed` 1-100 |
| `mycobot/coords_goal` | `MycobotSetCoords` | flange pose in `g_base`, mm / deg (`R = Rz·Ry·Rx`), IK with pinocchio |
| `mycobot/gripper_status` | `MycobotGripperStatus` | `true` open, `false` close |
| `mycobot/angles_real`, `mycobot/coords_real` | `MycobotAngles`, `MycobotCoords` | current state, 10 Hz |
| `cmd_vel` | `geometry_msgs/Twist` | forwarded (stamped) to `diff_drive_base_controller/cmd_vel` |

Examples:
```shell
ros2 topic pub --once /robot_0/mycobot/angles_goal mycobot_interfaces/msg/MycobotSetAngles \
  "{joint_1: 0.0, joint_2: 45.0, joint_3: -30.0, joint_4: 45.0, joint_5: 0.0, joint_6: 0.0, speed: 30}"
ros2 topic pub --once /robot_0/mycobot/coords_goal mycobot_interfaces/msg/MycobotSetCoords \
  "{x: 150.0, y: -60.0, z: 200.0, rx: -180.0, ry: 0.0, rz: -90.0, speed: 30, model: 0}"
ros2 topic pub --once /robot_0/mycobot/gripper_status mycobot_interfaces/msg/MycobotGripperStatus "{status: false}"
```

Gripper angles are parameters of `mycobot_sim_bridge` (`gripper_open` 0.15 rad, `gripper_closed` -0.5 rad, `gripper_time`). Commands are clamped to [-0.5, 0.15]: past -0.5 the linkage joints reach their own limits and the fingers fold into each other. Tune at runtime with
```shell
ros2 param set /robot_0/mycobot_sim_bridge gripper_closed -0.4
```

Arm keyboard teleoperation (in an `xterm`, like the base teleop):
```shell
ros2 launch limo_simulation arm_teleop.launch.py robot:=robot_0
```
Keys `q w e r t y` / `a s d f g h` jog joints 1-6 +/-, `o`/`c` open/close the gripper, `z` goes home, `+`/`-` change the step (default 5°).


## Author

[Federico Iadarola](https://github.com/fedeiada) 

## Credits
[Davide De Benedittis](https://github.com/ddebenedittis)

## References

- [`docker_ros_nvidia`](https://github.com/ddebenedittis/docker_ros_nvidia): base repo for this Docker container.