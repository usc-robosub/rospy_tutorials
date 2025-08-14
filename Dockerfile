# Use ROS Noetic base image for ARM64 (Raspberry Pi 4)
FROM ros:noetic-ros-base-focal

# Set environment variables for ROS
ENV ROS_DISTRO=noetic
ENV ROS_ROOT=/opt/ros/noetic
ENV ROS_PACKAGE_PATH=/opt/ros/noetic/share

# Install additional packages that might be useful
RUN apt-get update && apt-get install -y \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep if not already done
RUN rosdep update

# Create a workspace directory
COPY catkin_ws /catkin_ws
WORKDIR /catkin_ws

# Build the catkin workspace and source the setup
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Source both ROS and catkin workspace setup in bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

# Set ROS environment variables for external master
# These will be overridden by docker run -e flags
ENV ROS_MASTER_URI=http://192.168.2.76:11311
ENV ROS_HOSTNAME=192.168.2.1

# Default command
CMD ["bash", "-c", "source /opt/ros/noetic/setup.bash && bash"]
