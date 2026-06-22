# Algae Twin

Digital twin for a **physical TurtleBot3 Burger** (ROS 2 Jazzy + Gazebo Harmonic):
one Nav2 brain drives the real robot and its Gazebo twin in lockstep, operated from
a browser dashboard. Needs a real TurtleBot3 Burger on the network.

## Build

Needs ROS 2 Jazzy, Gazebo Harmonic, the turtlebot3 stack, Nav2 and `ros_gz`. Clone
into a colcon workspace that already has the turtlebot3 packages, then build:

```bash
cd ~/turtlebot3_ws/src
git clone https://github.com/aleksander-kolev/algae-twin-only.git algae_twin
cd ~/turtlebot3_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select algae_twin && source install/setup.bash
```

## Run

```bash
# on the robot Pi (same ROS_DOMAIN_ID — lab = 36):
ros2 launch turtlebot3_bringup robot.launch.py

# on the PC:
export ROS_DOMAIN_ID=36
ros2 launch algae_twin twin.launch.py     # add headless:=true on a weak GPU
ros2 run algae_twin preflight             # GO / NO-GO of every link
```

Open **http://localhost:8088**, set the robot pose once (UI *Set robot pose* or
RViz *2D Pose Estimate*), then place algae / block paths / E‑STOP from the dashboard.

## Notes

- Robot and PC need the same `ROS_DOMAIN_ID` and `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`.
- Use `headless:=true` without a GPU (software GL starves Nav2's control loop).

License: Apache‑2.0.
