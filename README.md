# ROS2 Foxy with limo repo for simulation 
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
./docker/build.bash [-a] [-f] [-h] [-r]
```
Where the optional arguments represent:
- `-a` or `--all`: build the image with all the dependencies
- `-f` or `--ffmpeg`: build the image with ffmpeg (for saving videos)
- `-h` or `--help`: show the help message
- `-r` or `--rebuild`: rebuild the image
- `-t` or `--torch`: build the image with PyTorch

Run the container with
```shell
./docker/run.bash
```

This repo also supports VS Code devcontainers.


## Author

[Federico Iadarola](https://github.com/fedeiada) 

## Credits
[Davide De Benedittis](https://github.com/ddebenedittis)

## References

- [`docker_ros_nvidia`](https://github.com/ddebenedittis/docker_ros_nvidia): base repo for this Docker container.