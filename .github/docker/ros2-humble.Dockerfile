# Dockerfile for ROS 2 Humble (Ubuntu 22.04)
# This image provides a base environment for running ROS 2 Humble nodes,
# Gazebo simulations, and Python-based AI components.
# It's intended for reproducible development and deployment.

ARG UBUNTU_VERSION=jammy
FROM osrf/ros:humble-desktop-${UBUNTU_VERSION}

LABEL maintainer="GIAIC Hackathon Q4 Team <your-email@example.com>"
LABEL description="Base image for AI Humanoid Robotics Book projects with ROS 2 Humble"

# Set environment variables for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV ROS_WS=/home/ros_user/ros2_ws

# Create a non-root user for security best practices
ARG USERNAME=ros_user
ARG USER_UID=1000
ARG USER_GID=1000

# Create the user and add to sudo group
RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    locales
RUN echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME}
RUN chmod 0440 /etc/sudoers.d/${USERNAME}
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Set up locale
RUN echo "en_US.UTF-8 UTF-8" > /etc/locale.gen && \
    locale-gen
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

# Install common build tools, development libraries, and ROS 2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    vim \
    wget \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-vcstool \
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-desktop \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    # Optional: For Isaac Sim bridge if needed (install separately)
    # ros-${ROS_DISTRO}-ros-ign-bridge \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init || true # Ignore error if already initialized
RUN rosdep update

# Install Python dependencies
COPY requirements.txt ${ROS_WS}/requirements.txt
RUN chown ${USERNAME}:${USERNAME} ${ROS_WS}/requirements.txt
USER ${USERNAME}
WORKDIR ${ROS_WS}
RUN python3 -m pip install --upgrade pip
RUN pip3 install -r requirements.txt

# Create ROS 2 workspace
RUN mkdir -p src
RUN colcon build --symlink-install

# Source ROS 2 setup files
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source ${ROS_WS}/install/setup.bash" >> ~/.bashrc

# Default command to run a bash shell
CMD ["/bin/bash"]