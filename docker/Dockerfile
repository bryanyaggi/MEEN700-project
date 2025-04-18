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
  curl \
  && rm -rf /var/lib/apt/lists/*

RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN apt-get update -qq && apt-get upgrade -y

# Create user
RUN groupadd -g $GID $UNAME
RUN useradd -u $UID -g $GID -ms /bin/bash $UNAME
RUN echo "$UNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER $UNAME
WORKDIR /home/$UNAME

ENV TERM xterm-256color
