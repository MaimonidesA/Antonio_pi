FROM ros:humble-ros-base

RUN sudo apt-get update \ 
    && apt-get install -y \
    nano \
    vim \
    git \
    && rm -rf /var/lib/apt/lists/*

# Installing rviz2
RUN sudo apt-get update \ 
    && apt-get install -y \
    ros-humble-rviz2 ros-humble-rqt \
    && rm -rf /var/lib/apt/lists/*

# Install Nav2 packages
RUN apt-get update && apt-get install -y \
   ros-$ROS_DISTRO-navigation2 \
   ros-$ROS_DISTRO-nav2-bringup \
   ros-$ROS_DISTRO-robot-localization \
   ros-$ROS_DISTRO-slam-toolbox \
   && rm -rf /var/lib/apt/lists/*

# Install realsense2 joint-state-publisher xacro packages
RUN apt-get update && apt-get install -y \
   ros-$ROS_DISTRO-realsense2-camera \
   ros-$ROS_DISTRO-imu-tools \
   ros-$ROS_DISTRO-joint-state-publisher-gui \
   ros-$ROS_DISTRO-xacro \
   && rm -rf /var/lib/apt/lists/*

# Install  ros-PCL
RUN apt-get update && apt-get install -y \
   ros-$ROS_DISTRO-pcl-ros \
   ros-$ROS_DISTRO-pcl-msgs \
   && rm -rf /var/lib/apt/lists/*

# Install OctoMap 
RUN apt-get update && apt-get install -y \
   ros-$ROS_DISTRO-octomap \
   ros-$ROS_DISTRO-octomap-msgs \
   ros-$ROS_DISTRO-octomap-mapping \
   ros-$ROS_DISTRO-octomap-rviz-plugins \
   && rm -rf /var/lib/apt/lists/*

# Install Xsens_MTi_ROS_Driver_and_Ntrip_Client 
RUN apt-get update && apt-get install -y \
   ros-$ROS_DISTRO-nmea-msgs \
   ros-$ROS_DISTRO-mavros-msgs \
   && rm -rf /var/lib/apt/lists/*

RUN sudo apt-get update \ 
   && apt-get install -y \
   ros-humble-teleop-twist-keyboard \
   && rm -rf /var/lib/apt/lists/*

# Example of copying a file
COPY config/ /site_config/

# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Set up sudo
RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

# Set up antonio_ws and OctoMap 
#  USER ros
#  RUN sudo apt-get update \
#  && . /opt/ros/$ROS_DISTRO/setup.sh \
#  && mkdir -p ~/antonio_ws/src \
#  && cd ~/antonio_ws \
#  && colcon build --symlink-install \
#  && cd ~/antonio_ws/src \
  #&& git clone https://github.com/OctoMap/octomap_ros.git --branch=ros2 \ 
  #&& git clone https://github.com/OctoMap/octomap_mapping.git --branch=ros2 \ 
#  && sudo rm -rf /var/lib/apt/lists/*

#USER root

# Copy the entrypoint and bashrc scripts so we have 
# our container's environment set up correctly
COPY entrypoint.sh /entrypoint.sh
COPY bashrc /home/${USERNAME}/.bashrc

# Set up entrypoint and default command
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]