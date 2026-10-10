# Local ROS 2 development environment.
FROM ros:humble-ros-base-jammy

# Add Ruff for Python formatting and linting.
COPY --from=ghcr.io/astral-sh/ruff:0.14.6 /ruff /usr/local/bin/ruff

# Additional sensor, Python and rosbridge dependencies.
RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    python3-pip \
    python3-numpy \
    python3-pytest \
    ros-humble-rosbridge-server \
    && rm -rf /var/lib/apt/lists/*

# Workspace and application dependencies.
WORKDIR /home/ros2_user/ros2_ws
COPY requirements.txt ./requirements.txt
RUN /usr/bin/python3 -m pip install --no-cache-dir \
    --target /opt/boat/python -r requirements.txt
ENV PYTHONPATH=/opt/boat/python

COPY setup_ros.sh /home/ros2_user/setup_ros.sh
RUN chmod +x /home/ros2_user/setup_ros.sh \
    && printf '\nsource /opt/ros/humble/setup.bash\nif [ -f /home/ros2_user/ros2_ws/install/setup.bash ]; then source /home/ros2_user/ros2_ws/install/setup.bash; fi\n' >> /etc/bash.bashrc

ENTRYPOINT ["/home/ros2_user/setup_ros.sh"]
CMD ["bash"]
