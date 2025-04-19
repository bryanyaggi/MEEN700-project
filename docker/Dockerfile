FROM osrf/ros:noetic-desktop-full

# Args for user
ARG UNAME=user
ARG UID=1000
ARG GID=1000

# Install system dependencies
RUN apt-get update && apt-get install -y \
  net-tools \
  iputils-ping \
  vim \
  ros-noetic-catkin \
  python3-catkin-tools \
  python3-pip \
  curl \
  && rm -rf /var/lib/apt/lists/*

# Add ROS apt repo
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Install ROS packages
RUN apt-get update && apt-get upgrade -y \
  ros-noetic-mapviz \
  ros-noetic-mapviz-plugins \
  ros-noetic-rviz-satellite

# Install Python packages
RUN pip3 install utm

# Create user
RUN groupadd -g $GID $UNAME
RUN useradd -u $UID -g $GID -ms /bin/bash $UNAME
RUN echo "$UNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER $UNAME
WORKDIR /home/$UNAME

ENV TERM xterm-256color
