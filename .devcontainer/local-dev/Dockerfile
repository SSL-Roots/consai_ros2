ARG ROS_DISTRO=humble

FROM osrf/ros:${ROS_DISTRO}-desktop as devcontainer

# Add vscode user with same UID and GID as your host system
# (copied from https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user)
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
# Switch from root to user
USER $USERNAME

# Set workdir
ENV ROS_OVERLAY=/home/${USERNAME}/ros2_ws
RUN mkdir -p $ROS_OVERLAY/src
WORKDIR $ROS_OVERLAY/src

# Install apt packages
RUN sudo apt update && sudo apt install -y \
    curl

# Install CONSAI dependent packages
RUN git clone https://github.com/SSL-Roots/consai_frootspi_msgs.git \
    && git clone https://github.com/SSL-Roots/frootspi_msgs.git \
    && git clone https://github.com/ssl-roots/rosbridge_suite.git \
    && sudo apt update \
    && rosdep update  \
    && rosdep install -r -y -i --from-paths . \
    && cd ../ \
    && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --symlink-install

# Copy source files
COPY --chown=${USERNAME}:${USERNAME} . consai_ros2

# Install consai_ros2
RUN cd consai_ros2/ \
    && rosdep install -r -y -i --from-paths . \
    && cd ../../ \
    && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --symlink-install

# Write sourcing the ROS setup file to .bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo "source /home/${USERNAME}/ros2_ws/install/setup.bash" >> ~/.bashrc
