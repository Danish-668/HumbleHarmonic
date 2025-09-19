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
    # NVIDIA GPU support libraries
    libglvnd0 \
    libglvnd-dev \
    libgl1 \
    libglx0 \
    libegl1 \
    libgles2 \
    && rm -rf /var/lib/apt/lists/*

# Add Gazebo packages repository (following official docs)
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install Gazebo Harmonic from binaries (as per official docs)
RUN apt-get update && apt-get install -y \
    gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# Remove any conflicting ros-gz packages first
RUN apt-get update && apt-get remove -y \
    ros-humble-ros-gz* \
    || true

# Install ROS-Gazebo Harmonic integration packages (official approach)
RUN apt-get update && apt-get install -y \
    ros-humble-ros-gzharmonic \
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
    && rm -rf /var/lib/apt/lists/*

# Create gz_ros2_control workspace and clone source (following official docs)
RUN mkdir -p /opt/gz_ros2_control_ws/src && \
    cd /opt/gz_ros2_control_ws/src && \
    git clone https://github.com/ros-controls/gz_ros2_control.git -b humble

# Build gz_ros2_control from source
RUN /bin/bash -c "cd /opt/gz_ros2_control_ws && \
    source /opt/ros/humble/setup.bash && \
    export GZ_VERSION=harmonic && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"

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

# Install rosdep rules (required for Gazebo Harmonic packages)
RUN rosdep init || echo "rosdep already initialized" && \
    rosdep update --rosdistro humble

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
    echo "# gz_ros2_control workspace (built from source)" >> ~/.bashrc && \
    echo "source /opt/gz_ros2_control_ws/install/setup.bash" >> ~/.bashrc && \
    echo "" >> ~/.bashrc && \
    echo "# Workspace paths" >> ~/.bashrc && \
    echo "export GZ_SIM_RESOURCE_PATH=/opt/gz_ros2_control_ws/install/share/gz_ros2_control:/workspace/humble_harmonic/install/share:/workspace/humble_harmonic/src:\$GZ_SIM_RESOURCE_PATH" >> ~/.bashrc && \
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