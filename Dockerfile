FROM ros:noetic-ros-base-focal

WORKDIR /catkin_ws

COPY catkin_ws /catkin_ws

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc && \
    sed -i '1iforce_color_prompt=yes' ~/.bashrc

# Default command
CMD ["bash", "-c", "bash"]
