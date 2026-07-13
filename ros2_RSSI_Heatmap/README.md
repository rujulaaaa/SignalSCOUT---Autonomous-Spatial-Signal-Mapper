# SignalSCOUT — Autonomous Spatial Signal Mapper

## Problem Statement

Reliable wireless connectivity is essential in modern offices, hospitals, warehouses, campuses, factories, and smart buildings. However, despite the importance of stable network coverage, evaluating WiFi performance still relies heavily on manual site surveys. Engineers must physically walk through an environment with measurement devices, stopping at numerous locations to record signal strength and identify areas with poor connectivity.

This process is time-consuming, labor-intensive, and often difficult to reproduce consistently. Large facilities may require hundreds of measurement points, making surveys expensive to conduct and challenging to repeat whenever the network layout changes. Moreover, manual sampling may overlook certain regions or produce inconsistent results due to variations in the survey path.

This project explores an autonomous alternative to traditional wireless site surveys. Instead of relying on human operators, an autonomous mobile robot performs the entire data collection process. The robot builds a map of its surroundings, navigates safely through the environment, records simulated WiFi signal strength (RSSI) at strategically distributed locations, and compiles the collected measurements into a spatial visualization of wireless coverage. By automating the survey process, the system enables faster, repeatable, and scalable network assessment while significantly reducing human effort.

---

## The Story

Imagine you're the network administrator of a large office building where employees frequently complain about unstable WiFi connections. Some meeting rooms experience frequent disconnections, certain hallways have noticeably weaker signals, and a few workspaces seem to be complete dead zones. Although multiple access points have been installed throughout the building, there is no clear understanding of how the wireless signal propagates across the entire floor.

Traditionally, investigating these issues would require an engineer to manually walk through every corridor, office, and workspace with specialized equipment, recording signal strength measurements one location at a time. The process can take hours, and repeating the survey after changes to the network infrastructure is equally demanding.

Instead, an autonomous mobile robot takes over the task.

The robot first explores the environment using **SLAM**, constructing a map while simultaneously estimating its own position. Once a reliable map has been created, it localizes itself within that map and automatically generates a dense set of safe navigation waypoints that provide comprehensive coverage of the accessible area.

The robot then begins its autonomous survey. It systematically navigates from one waypoint to the next, estimating the received WiFi signal strength (RSSI) at each location and storing every measurement together with its precise coordinates. Without requiring human supervision, it continues until the entire environment has been surveyed.

After the survey is complete, the collected measurements are interpolated to generate a continuous wireless coverage heatmap. Rather than displaying isolated data points, the final visualization provides an intuitive overview of signal distribution, clearly highlighting regions with strong coverage, weak connectivity, and potential dead zones. Network administrators can use this information to evaluate access point placement, identify problematic areas, and make informed decisions to improve overall wireless performance.

> By combining autonomous navigation, mapping, localization, wireless signal sampling, and spatial visualization into a single workflow, the project demonstrates how robotics can transform traditional, manual network site surveys into an efficient, repeatable, and intelligent autonomous solution.


## Objective

Develop a complete autonomous ROS 2 pipeline capable of:

- Building a map using SLAM
- Localizing within the environment
- Generating safe navigation waypoints
- Visiting every waypoint using Nav2
- Measuring simulated WiFi RSSI
- Collecting all measurements
- Generating wireless signal heatmaps

**Overall pipeline:**

```
Map → Localize → Navigate → Measure → Aggregate → Visualize
```

---

## System Overview

The project consists of four major components.

### 1. Waypoint Publisher
Generates uniformly distributed collision-free waypoints across the occupancy map and sends them to Nav2 in batches for autonomous navigation.

### 2. RSSI Measurement Plugin
Runs as a Nav2 waypoint task plugin. Every time the robot reaches a waypoint, the plugin estimates the WiFi signal strength using a simulated log-distance propagation model and publishes the measurement.

### 3. RSSI Simulator
Implements a simplified wireless propagation model. The received signal strength depends on:
- Robot position
- Access Point position
- Path-loss exponent
- Transmit power
- Gaussian noise

### 4. Heatmap Generator
Receives RSSI measurements together with robot positions. After the survey is complete, it interpolates the sparse measurements into continuous RSSI heatmaps and saves the final visualization.

---

## What You Need To Implement

This repository contains several TODOs that must be completed.

### 1. Heatmap Generator — `rssi_heatmap_generator_package/rssi_heatmap_generator_package/node.py`

**TODO 1 — Coordinate Conversion**

Convert robot positions received in the ROS map frame (meters) into image pixel coordinates using the map origin and map resolution. These pixel coordinates will later be used for waypoint visualization and heatmap interpolation.

**TODO 2 — Heatmap Generation**

Generate two RSSI heatmaps from the collected measurements:
- **Absolute RSSI Heatmap**
- **Relative RSSI Heatmap**

These heatmaps estimate wireless signal strength across the entire environment by interpolating discrete waypoint measurements.

**TODO 3 — Visualization and Export**

Overlay the generated heatmaps on the occupancy map. Display the results and save:
- Waypoint map
- Absolute RSSI heatmap
- Relative RSSI heatmap

…using timestamped filenames.

---

### 2. RSSI Measurement Plugin — `nav2_read_rssi_at_waypoint_plugin/src/read_rssi_at_waypoint.cpp`

**TODO 1**

After reaching every waypoint:
- Collect multiple simulated RSSI measurements
- Ignore invalid readings
- Compute the average RSSI
- Store the averaged value in the outgoing ROS message

> Averaging multiple samples produces a more stable signal estimate before publishing it.

---

### 3. RSSI Simulator — `nav2_read_rssi_at_waypoint_plugin/include/simulated_rssi.hpp`

**TODO 1**

Implement the wireless propagation model by:
- Computing RSSI using the **log-distance path-loss equation**
- Adding Gaussian noise
- Clamping the result to the valid RSSI range
- Returning the simulated RSSI value

---

### 4. Waypoint Publisher — `waypoint_publisher_package/waypoint_publisher_package/node.py`

**TODO 1 — Generate Safe Waypoints**

- Identify all navigable cells in the occupancy map
- Generate waypoints only in free space while avoiding locations too close to obstacles
- Convert valid pixel coordinates into ROS map coordinates

**TODO 2 — Send Navigation Goals**

Convert generated waypoints into `PoseStamped` messages and send them to the Nav2 `FollowWaypoints` action server in batches. Continue sending batches until every waypoint has been visited.

---

## Running the Project

### Prerequisites 

**Nav2 and Dependencies**

```bash
sudo apt install -y \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-bt-navigator \
  ros-jazzy-nav2-controller \
  ros-jazzy-nav2-planner \
  ros-jazzy-nav2-behaviors \
  ros-jazzy-nav2-waypoint-follower \
  ros-jazzy-nav2-lifecycle-manager \
  ros-jazzy-nav2-regulated-pure-pursuit-controller \
  ros-jazzy-nav2-costmap-2d \
  ros-jazzy-nav2-smoother \
  ros-jazzy-nav2-velocity-smoother \
  ros-jazzy-slam-toolbox \
  ros-jazzy-cartographer-ros
```

**Install TurtleBot3 Packages**

This project requires the TurtleBot3 packages for simulation. Install them before proceeding.
```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-turtlebot3 \
  ros-jazzy-turtlebot3-simulations \
  ros-jazzy-turtlebot3-gazebo
```

**Twist Stamper Packages**

The TurtleBot3 Gazebo bridge expects `geometry_msgs/msg/TwistStamped` on `/cmd_vel`, but Nav2 publishes `geometry_msgs/msg/Twist`. The `twist_stamper` package converts between them.

```bash
sudo apt install ros-jazzy-twist-stamper
```

> **Set up the TurtleBot3 workspace (if building from source):**
> ```bash
> mkdir -p ~/turtlebot3_ws/src
> cd ~/turtlebot3_ws/src
> git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git
> git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
> ```

---

**Set the default TurtleBot3 model** (add to your `~/.bashrc` for persistence):
```bash
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
source ~/.bashrc
```

### Step 1 — Launch Gazebo

**Build the Turtlebot3 workspace**
```bash
cd ~/turtlebot3_ws
colcon build
```

**Launch the turtlebot3 model**

Terminal 1
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Wait until Gazebo loads completely.

---

### Step 2 — Launch SLAM & Build a Map

**Launch SLAM:**

Terminal 2
```bash
cd ~/ros2_RSSI_Heatmap
colcon build
source /opt/ros/jazzy/setup.bash
source ~/turtlebot3_ws/install/setup.bash
source ~/ros2_RSSI_Heatmap/install/setup.bash
ros2 launch mappers_bringup sim_nav2_slam.launch.py
```

**Open another terminal for teleoperation:**

Terminal 3
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
> Or use the default Gazebo Teleop GUI

Drive the robot around until the environment is completely mapped.

**Save the generated map:**
```bash
mkdir -p ~/ros2_RSSI_Heatmap/src/maps
ros2 run nav2_map_server map_saver_cli \
  -f ~/ros2_RSSI_Heatmap/src/maps/map \
  --ros-args -p use_sim_time:=true
```

This creates:
```
maps/
├── map.yaml
└── map.pgm
```

Stop SLAM and teleoperation after saving the map.

---

### Step 3 — Launch Navigation

Terminal 2 (after closing SLAM)
Start localization and the Nav2 stack:
```bash
ros2 launch mappers_bringup sim_nav2_localization.launch.py \
  map:=$HOME/ros2_RSSI_Heatmap/src/maps/map.yaml
```

Wait about **10–15 seconds** for all lifecycle nodes to activate.

---

### Step 4 — Run the Autonomous Survey

Launch the waypoint publisher and heatmap generator:
```bash
ros2 launch mappers_bringup mappers.launch.py \
  path_to_yaml:=$HOME/ros2_RSSI_Heatmap/src/maps/map.yaml
```

The robot will now:
1. Generate safe waypoints
2. Navigate autonomously
3. Collect RSSI measurements
4. Generate wireless heatmaps

**Example output:**
```
[heatmap_generator]: Received RSSI=-46 at pixel (57,14)
[heatmap_generator]: Received RSSI=-49 at pixel (65,14)
...
[waypoint_publisher]: All batches complete — publishing trigger...
[heatmap_generator]: Generating heatmap...
[heatmap_generator]: Heatmaps saved successfully.
```

Generated images are stored in `~/ros2_RSSI_Heatmap/src/heatmaps/`, including:
```
waypoints_YYYYMMDD_HHMMSS.png
heatmap_abs_YYYYMMDD_HHMMSS.png
heatmap_rel_YYYYMMDD_HHMMSS.png
```
---

## Bonus Challenge 1 — Multi-Access Point Support

### Objective

Currently, the RSSI simulator models signal strength from a single WiFi Access Point.

Modify the project so the environment supports **multiple Access Points**, allowing the robot to evaluate wireless coverage in more realistic scenarios where signals from different APs overlap. The generated heatmaps should reflect the strongest available signal (or combined coverage) at every surveyed location.

### What to implement

Create or extend a node that:
- Loads multiple Access Point positions and transmission parameters from a YAML file or ROS parameters.
- Computes the RSSI at each waypoint by evaluating the signal received from every Access Point and selecting the strongest (or combined) signal.
- Publishes the updated RSSI measurements while remaining compatible with the existing waypoint navigation pipeline.
- Generates wireless coverage heatmaps that visualize the effect of multiple Access Points and reports basic statistics, such as the Access Point serving each waypoint and overall coverage quality.

## Bonus Challenge 2 — Nearest Waypoint First Navigation

### Objective

Currently, the robot visits generated waypoints in their original order, which may result in long and inefficient travel paths.

Modify the project so the robot always selects the **nearest unvisited waypoint** as its next navigation goal, reducing overall travel distance and improving survey efficiency.

### What to implement

Create or extend a node that:
- Computes the distance from the robot's current position to all unvisited waypoints.
- Selects the nearest reachable waypoint as the next navigation target.
- Continuously updates the waypoint list until all locations have been visited.

---

## Deliverables

### 1. Source Code
- Completed implementations for all TODOs
- Working ROS 2 packages
- Launch files

### 2. Demonstration Video

Show:
- Gazebo simulation
- SLAM map creation
- Autonomous waypoint navigation
- RSSI data collection
- Final generated heatmaps

### 3. Report

Briefly describe:
- Waypoint generation strategy
- RSSI simulation model
- Heatmap interpolation approach
- Overall system workflow
- Challenges faced and improvements made

---

## Final Message

WiFi dead zones don’t speak up — they have to be discovered.

This project turns that job over to a robot that doesn’t just move through space, but reads it. It senses signal strength, builds a map as it goes, and transforms invisible connectivity gaps into a clear, actionable heatmap.

Complete the system, and you won’t just have a robot that navigates a room — you’ll have one that understands it well enough to reveal what humans can’t easily see.

That’s the shift from robotics as movement… to robotics as insight.
