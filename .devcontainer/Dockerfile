FROM osrf/ros:humble-desktop

# Install basic development tools
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    curl \
    lsb-release \
    gnupg \
    && rm -rf /var/lib/apt/lists/*

# Install Gazebo Harmonic
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update && apt-get install -y \
    gz-harmonic \
    ros-humble-ros-gz \
    && rm -rf /var/lib/apt/lists/*

# Source ROS and Gazebo setup in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

WORKDIR /workspaces