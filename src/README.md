
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

These packages have been tested with ROS 2 Foxy on an Ubuntu system.


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
```

Build the workspace with
```shell
colcon build --symlink-install
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
ros2 launch limo_simulation gazebo_models_diff.launch.py n_robots:=<put you number>
```
this will spawn n robots depending on the argument. Robot are differential drive
Then you can develop your own algorithm and test it in gazebo.
Inside src folder, there is a simple diff_drive_publisher BUT be carefull to adjust the namespacing if you are spawning more robots


## Author

- [Federico Iadarola](https://github.com/fedeiada)
