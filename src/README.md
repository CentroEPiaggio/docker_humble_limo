
ROS 2 packages to implement generic ros2-controllers based with limo simulation.

## Table of Contents

- [Table of Contents](#table-of-contents)
- [Overview](#overview)
- [Installation](#installation)
- [Usage](#usage)
  - [First time](#first-time)
  - [develop](#develop)
  - [Scripts](#scripts)
    - [Examples](#examples)
- [Author](#author)

## Overview

## Installation

These packages have been tested with ROS 2 Humble and ROS 2 Iron on an Ubuntu system.

To use Torch with an NVIDIA graphics card, it is necessary to install the NVIDIA drivers for Ubuntu. [Here](https://letmegooglethat.com/?q=Install+nvidia+drivers+ubuntu).

## Usage
### First time 
Build the workspace with limo package with
```shell
colcon build --symlink-install
```

Source the workspace with (you have to add it to the `~/.bashrc` or do it on every newly opened terminal)
```shell
source install/setup.bash
```
### develop
For new workspace  create folder and clone the repo in the source directory (don't  call your workspace a generic `<my_workspace>`)
```shell
mkdir <my_workspace>
cd <my_workspace>
git clone <whatever you want>
cd ..
```

Build the whole workspace with
```shell
colcon build --symlink-install
```
or just the specific package with
```shell
colcon build --packages-select <my_workspace> --symlink-install
```

Source the workspace with (you have to add it to the `~/.bashrc` or do it on every newly opened terminal)
```shell
source install/setup.bash
```

### Scripts

- limo_simulation
  provide a gazebo simulation for testing limo models in gazebo.  

#### Examples
```shell
ros2 launch limo_simulation gazebo_models_diff.launch.py
```
this will spawn n robots depending on the position passed in the launch file. Robot are differential drive
Then you can develop your own algorithm and test it in gazebo.
Inside src folder, there is a simple diff_drive_publisher BUT be carefull to adjust the namespacing if you are spawning more robots


## Author

- [Federico Iadarola](https://github.com/fedeiada)
