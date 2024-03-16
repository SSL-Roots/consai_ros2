ARG ROS_DISTRO=humble

FROM ros:${ROS_DISTRO} as devcontainer

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

# Install apt packages
RUN sudo apt update && sudo apt install -y \
    curl \
    && sudo apt clean \
    && sudo rm -rf /var/lib/apt/lists/*

# Install nodejs for consai_web_ui
RUN curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash - &&\
    sudo apt-get install -y nodejs

# Create ROS2 Workspace
RUN mkdir -p /home/${USERNAME}/ros2_ws/src

# Write sourcing the ROS setup file to .bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo "source /home/${USERNAME}/ros2_ws/install/setup.bash" >> ~/.bashrc

# Copy source files
COPY . /home/${USERNAME}/ros2_ws/src/consai_ros2

# Install CONSAI dependent packages
RUN cd /home/${USERNAME}/ros2_ws/src \
    && git clone https://github.com/SSL-Roots/consai_frootspi_msgs.git \
    && git clone https://github.com/SSL-Roots/frootspi_msgs.git \
    && git clone https://github.com/ssl-roots/rosbridge_suite.git \
    && sudo apt update \
    && rosdep update  \
    && rosdep install -r -y -i --from-paths .

# Build the workspace
RUN cd /home/${USERNAME}/ros2_ws \
    && . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --symlink-install
