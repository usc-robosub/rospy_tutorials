FROM ros:humble-ros-base-jammy

WORKDIR /ros2_ws

COPY ros2_ws/src /ros2_ws/src

RUN rosdep update && \
    apt-get update && \
    rosdep install --from-paths src --ignore-src -y

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc \
    && sed -i '1iforce_color_prompt=yes' ~/.bashrc

ENTRYPOINT ["/entrypoint.sh"]

CMD ["bash"]