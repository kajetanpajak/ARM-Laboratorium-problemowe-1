# ArUco Cube Search

Requires the ROS 2 Jazzy laboratory environment with `arm05_sim` and Nav2.

## 1. Clone
```bash
source /opt/ros/jazzy/setup.bash
source /arm_ws/install/setup.bash

cd /arm_ws/src
git clone https://github.com/kajetanpajak/ARM-Laboratorium-problemowe-1.git
```

## 2. Install dependencies

```bash
cd /arm_ws
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
```

## 3. Build

```bash
cd /arm_ws
colcon build --symlink-install \
  --packages-select cube_search_interfaces cube_search

source /arm_ws/install/setup.bash
```

## 4. Configure the map

In the `map.yaml` loaded by the simulation, set `free_thresh` to the
tested value that keeps free and unknown regions separate:

```yaml
free_thresh: 0.196
```

## 5. Launch

```bash
ros2 launch cube_search bringup.launch.py
```

This starts the simulation, RViz, detector, and search server.

## 6. Send a goal

Send a goal in a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
source /arm_ws/install/setup.bash

ros2 action send_goal \
  /search_aruco_cubes \
  cube_search_interfaces/action/SearchCubes \
  "{expected_count: 5}" \
  --feedback
```
