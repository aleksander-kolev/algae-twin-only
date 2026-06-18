# Algae Twin — TWIN-ONLY build. Reproducible ROS 2 Jazzy environment for the
# real TurtleBot3 Burger + its Gazebo twin (one shared Nav2 brain).
#
# The operator UI is a browser dashboard served over the stdlib (no GUI library)
# — open the printed http://localhost:8088 . `xvfb` gives Gazebo's gpu_lidar a
# software GL context so the twin's /sim/scan is produced with no display.
#
#   docker build -t algae-twin-only .
#   # twin mode needs the robot's bringup reachable on the SAME ROS_DOMAIN_ID +
#   # RMW, so run with host networking so DDS discovery can reach the robot:
#   docker run -it --rm --network host -e ROS_DOMAIN_ID=36 \
#     -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp algae-twin-only \
#     bash -lc 'ros2 launch algae_twin twin.launch.py'
#
# (Cartographer / the re-mapping workflow is intentionally not included in this
# twin-only build.)
FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    git build-essential python3-colcon-common-extensions \
    ros-jazzy-ros-gz \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-map-server \
    ros-jazzy-xacro \
    ros-jazzy-rmw-fastrtps-cpp \
    xvfb \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/twin_ws
RUN mkdir -p src && cd src && \
    git clone --depth 1 -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git && \
    git clone --depth 1 -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git && \
    git clone --depth 1 -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

COPY algae_twin /opt/twin_ws/src/algae_twin

SHELL ["/bin/bash", "-lc"]

RUN source /opt/ros/jazzy/setup.bash && \
    apt-get update && rosdep update && \
    rosdep install --rosdistro jazzy --from-paths src --ignore-src -r -y || true && \
    rm -rf /var/lib/apt/lists/* && \
    colcon build --packages-select \
      turtlebot3_msgs turtlebot3_description turtlebot3_gazebo \
      turtlebot3_teleop algae_twin

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /opt/twin_ws/install/setup.bash" >> /root/.bashrc && \
    echo "export TURTLEBOT3_MODEL=burger" >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=36" >> /root/.bashrc

CMD ["/bin/bash"]
