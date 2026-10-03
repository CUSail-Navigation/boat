# Local ROS 2 development environment.
FROM ros:lyrical-ros-base-resolute

# Add Ruff for Python formatting and linting.
COPY --from=ghcr.io/astral-sh/ruff:0.14.6 /ruff /usr/local/bin/ruff

# Compiler, ROS tooling, lint checks and dashboard connectivity.
RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-pip \
    python3-numpy \
    python3-pytest \
    python3-rosdep \
    ros-lyrical-ament-cmake-pytest \
    ros-lyrical-ament-copyright \
    ros-lyrical-ament-flake8 \
    ros-lyrical-ament-pep257 \
    ros-lyrical-ament-lint-auto \
    ros-lyrical-ament-lint-common \
    ros-lyrical-rosbridge-server \
    && rm -rf /var/lib/apt/lists/*

# Workspace and application dependencies.
WORKDIR /home/ros2_user/ros2_ws
COPY requirements.txt ./requirements.txt
RUN /usr/bin/python3 -m pip install --no-cache-dir \
    --target /opt/boat/python -r requirements.txt
ENV PYTHONPATH=/opt/boat/python

COPY setup_ros.sh /home/ros2_user/setup_ros.sh
RUN chmod +x /home/ros2_user/setup_ros.sh \
    && printf '\nsource /opt/ros/lyrical/setup.bash\nif [ -f /home/ros2_user/ros2_ws/install/setup.bash ]; then source /home/ros2_user/ros2_ws/install/setup.bash; fi\n' >> /etc/bash.bashrc

ENTRYPOINT ["/home/ros2_user/setup_ros.sh"]
CMD ["bash"]
