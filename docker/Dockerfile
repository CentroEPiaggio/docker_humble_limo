# Change ROS version in build.bassh, not here!
ARG BASE_IMAGE=osrf/ros
ARG BASE_TAG=noetic-desktop

FROM ${BASE_IMAGE}:${BASE_TAG}

# ============================================================================ #

# Prevent bash to ask for user input which may break the building process
ENV DEBIAN_FRONTEND=noninteractive

# Install sudo, and some essential packages.
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt --mount=type=cache,sharing=locked,target=/var/lib/apt \
    apt-get update && apt-get install --no-install-recommends -qqy \
    bash-completion \
    build-essential \
    git \
    python3-pip \
    ros-$ROS_DISTRO-gazebo-ros-pkgs \
    ros-$ROS_DISTRO-ros-ign-bridge \
    ros-$ROS_DISTRO-gazebo-ros2-control \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-tf-transformations \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-pinocchio \
    ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-rqt-graph \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-xacro \
    sudo \
    xterm

# Install the python packages.
RUN pip3 install \
    casadi \
    matplotlib \
    numpy \
    pandas \
    pre-commit \
    progressbar2 \
    qpsolvers \
    quadprog \
    ruff \
    scipy \
    transforms3d \
    --upgrade

ARG TORCH=0
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt --mount=type=cache,sharing=locked,target=/var/lib/apt \
    if [ "${TORCH}" = "1" ] ; then \
        apt-get update && apt-get install --no-install-recommends -qqy \
        python3-torch \
        python3-torchaudio \
        python3-torchvision ; \
    fi

ARG FFMPEG=0
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt --mount=type=cache,sharing=locked,target=/var/lib/apt \
    if [ "${FFMPEG}" = "1" ] ; then \
        apt-get update && apt-get install --no-install-recommends -qqy \
        ffmpeg ; \
    fi

# Install Latex
ARG LATEX=0
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt --mount=type=cache,sharing=locked,target=/var/lib/apt \
    if [ "${LATEX}" = "1" ] ; then \
        apt-get update && apt-get install --no-install-recommends -qqy \
        dvipng \
        texlive-latex-extra \
        texlive-fonts-recommended \
        texlive-fonts-extra \
        cm-super ; \
    fi

# Create the same user as the host itself. (By default Docker creates the container as root, which is not recommended.)
ARG UID=1000
ARG GID=1000
ARG USER=ros
ARG PWDR=/
RUN addgroup --gid ${GID} ${USER} \
 && adduser --gecos "ROS User" --disabled-password --uid ${UID} --gid ${GID} ${USER} \
 && usermod -a -G dialout ${USER} \
 && echo ${USER}" ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers.d/99_aptget \
 && chmod 0440 /etc/sudoers.d/99_aptget && chown root:root /etc/sudoers.d/99_aptget

# Choose to run as user
ENV USER=${USER}
USER ${USER}

# Change HOME environment variable
ENV HOME=/home/${USER}

# Set up environment
COPY ./docker/.config/update_bashrc_ros_2 /sbin/update_bashrc_ros_2

RUN sudo chmod +x /sbin/update_bashrc_ros_2 ; sudo chown ${USER} /sbin/update_bashrc_ros_2 \
    && echo 'echo "source '${PWDR}'/install/setup.bash" >> ~/.bashrc' >> /sbin/update_bashrc_ros_2 \
    && cat /sbin/update_bashrc_ros_2 \
    && sync ; /bin/bash -c /sbin/update_bashrc_ros_2 ; sudo rm /sbin/update_bashrc_ros_2 ; 

# Change entrypoint to source ~/.bashrc and start in ~
COPY ./docker/.config/entrypoint.sh /ros_entrypoint.sh
RUN sudo chmod +x /ros_entrypoint.sh ; sudo chown ${USER} /ros_entrypoint.sh \
 && echo "cd "${PWDR} >> /ros_entrypoint.sh \
 && echo 'exec bash -i -c $@' >> /ros_entrypoint.sh \
 && cat /ros_entrypoint.sh

ENV MPLCONFIGDIR=/home/${USER}/.matplotlib

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
