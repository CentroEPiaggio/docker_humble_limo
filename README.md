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


## Author

[Federico Iadarola](https://github.com/fedeiada) 

## Credits
[Davide De Benedittis](https://github.com/ddebenedittis)

## References

- [`docker_ros_nvidia`](https://github.com/ddebenedittis/docker_ros_nvidia): base repo for this Docker container.