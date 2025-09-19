#!/bin/bash

# Script to verify and set up NVIDIA GPU support for Docker and Gazebo

echo "=========================================="
echo "NVIDIA GPU Setup for ROS2 + Gazebo Harmonic"
echo "=========================================="
echo ""

# Check for NVIDIA driver
echo "1. Checking NVIDIA driver..."
if command -v nvidia-smi &> /dev/null; then
    nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv
    echo "NVIDIA driver installed"
else
    echo "NVIDIA driver not found. Please install NVIDIA drivers first."
    echo "   Visit: https://www.nvidia.com/Download/index.aspx"
    exit 1
fi
echo ""

# Check for Docker
echo "2. Checking Docker..."
if command -v docker &> /dev/null; then
    docker --version
    echo "Docker installed"
else
    echo "Docker not found. Please install Docker first."
    exit 1
fi
echo ""

# Check for NVIDIA Container Toolkit
echo "3. Checking NVIDIA Container Toolkit..."
if docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi &> /dev/null; then
    echo "NVIDIA Container Toolkit is working"
else
    echo "NVIDIA Container Toolkit might not be installed or configured."
    echo ""
    echo "To install NVIDIA Container Toolkit on Ubuntu/Debian:"
    echo ""
    echo "distribution=\$(. /etc/os-release;echo \$ID\$VERSION_ID)"
    echo "curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -"
    echo "curl -s -L https://nvidia.github.io/nvidia-docker/\$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list"
    echo "sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit"
    echo "sudo systemctl restart docker"
    echo ""
    exit 1
fi
echo ""

# Test GPU in container
echo "4. Testing GPU access in container..."
docker run --rm --gpus all --ipc=host ubuntu:22.04 bash -c "apt-get update && apt-get install -y mesa-utils > /dev/null 2>&1 && glxinfo | grep 'OpenGL renderer'"
echo ""

echo "=========================================="
echo "GPU Setup Complete!"
echo "=========================================="
echo ""
echo "To rebuild the DevContainer with GPU support:"
echo "1. Open VS Code"
echo "2. Press Ctrl+Shift+P"
echo "3. Run: 'Dev Containers: Rebuild Container'"
echo ""
echo "To verify GPU in Gazebo after rebuild:"
echo "1. Launch Gazebo: gz sim"
echo "2. Check rendering: gz topic -e /stats | grep real_time_factor"
echo "   (GPU should give higher real_time_factor values)"