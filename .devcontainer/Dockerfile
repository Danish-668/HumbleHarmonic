FROM osrf/ros:humble-desktop-full-jammy

# Install basic development tools and GUI dependencies
RUN apt-get update && apt-get install -y \
    git \
    vim \
    nano \
    wget \
    curl \
    build-essential \
    cmake \
    python3-pip \
    python3-colcon-common-extensions \
    lsb-release \
    gnupg \
    software-properties-common \
    # GUI support packages
    x11-apps \
    xauth \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libxcb-xfixes0-dev \
    libxrender-dev \
    libxi-dev \
    libxkbcommon-dev \
    libxkbcommon-x11-dev \
    # Qt dependencies for Gazebo GUI
    qt6-base-dev \
    libqt6core6 \
    libqt6gui6 \
    libqt6widgets6 \
    libqt6opengl6 \
    # Additional tools
    xterm \
    && rm -rf /var/lib/apt/lists/*

# Add Gazebo packages repository (following official docs)
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install Gazebo Harmonic from binaries (as per official docs)
RUN apt-get update && apt-get install -y \
    gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 packages needed for gz_ros2_control
RUN apt-get update && apt-get install -y \
    ros-humble-control-msgs \
    ros-humble-control-toolbox \
    ros-humble-controller-interface \
    ros-humble-controller-manager \
    ros-humble-hardware-interface \
    ros-humble-pluginlib \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-realtime-tools \
    ros-humble-xacro \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-rviz2 \
    ros-humble-teleop-twist-keyboard \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-image \
    && rm -rf /var/lib/apt/lists/*

# Install Gazebo development libraries needed for gz_ros2_control
RUN apt-get update && apt-get install -y \
    libgz-sim8-dev \
    libgz-plugin2-dev \
    libgz-transport13-dev \
    libgz-msgs10-dev \
    libgz-math7-dev \
    libgz-cmake3-dev \
    libgz-common5-dev \
    libgz-tools2-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install \
    setuptools==58.2.0 \
    pytest \
    pytest-cov

# Install dependencies for Claude Code
RUN apt-get update && apt-get install -y \
    ca-certificates \
    gnupg \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# Install Node.js 20 (required for Claude Code)
RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash - && \
    apt-get install -y nodejs && \
    rm -rf /var/lib/apt/lists/*

# Install Claude Code from npm
RUN npm install -g @anthropic-ai/claude-code && \
    echo "Claude Code installed successfully"

# Create workspace directories for gz_ros2_control
RUN mkdir -p /opt/gz_ros2_control_ws/src

# Clone and build gz_ros2_control from source (as per official docs)
WORKDIR /opt/gz_ros2_control_ws/src
RUN git clone https://github.com/ros-controls/gz_ros2_control.git -b humble

WORKDIR /opt/gz_ros2_control_ws
# Build gz_ros2_control
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF"

# Create main workspace directory
RUN mkdir -p /workspace/humble_harmonic

# Setup environment in bashrc
RUN echo "# Claude CLI Path" >> ~/.bashrc && \
    echo "export PATH=\"/root/.local/bin:\$PATH\"" >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    echo "# ROS 2 Humble" >> ~/.bashrc && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    echo "# Gazebo Harmonic" >> ~/.bashrc && \
    echo "export GZ_VERSION=harmonic" >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    echo "# gz_ros2_control" >> ~/.bashrc && \
    echo "source /opt/gz_ros2_control_ws/install/setup.bash" >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    echo "# Workspace paths" >> ~/.bashrc && \
    echo "export GZ_SIM_RESOURCE_PATH=/workspace/humble_harmonic/install/share:/workspace/humble_harmonic/src:\$GZ_SIM_RESOURCE_PATH" >> ~/.bashrc && \
    echo "export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/gz_ros2_control_ws/install/gz_ros2_control/lib:\$GZ_SIM_SYSTEM_PLUGIN_PATH" >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    echo "# Source local workspace if it exists" >> ~/.bashrc && \
    echo "if [ -f /workspace/humble_harmonic/install/setup.bash ]; then" >> ~/.bashrc && \
    echo "    source /workspace/humble_harmonic/install/setup.bash" >> ~/.bashrc && \
    echo "fi" >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    echo "# Set ROS domain" >> ~/.bashrc && \
    echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc && \
    echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    echo "# GUI/Display settings" >> ~/.bashrc && \
    echo "export DISPLAY=\${DISPLAY:-:0}" >> ~/.bashrc && \
    echo "export QT_X11_NO_MITSHM=1" >> ~/.bashrc && \
    echo "export LIBGL_ALWAYS_INDIRECT=0" >> ~/.bashrc && \
    echo "export LIBGL_ALWAYS_SOFTWARE=0" >> ~/.bashrc && \
    echo "export QT_QPA_PLATFORM=xcb" >> ~/.bashrc

# Set the default working directory
WORKDIR /workspace/humble_harmonic

# Set up entrypoint
SHELL ["/bin/bash", "-c"]
CMD ["/bin/bash"]