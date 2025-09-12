# Use ROS Noetic base image for ARM64 (Raspberry Pi 4)
FROM ros:noetic-ros-base-focal

# Create a workspace directory
COPY catkin_ws /catkin_ws
WORKDIR /catkin_ws

# Build the catkin workspace and source the setup
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Source both ROS and catkin workspace setup in bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

# Default command
CMD ["bash", "-c", "bash"]
