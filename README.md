
# README.md

## Overview

This workspace demonstrates an end‐to‐end pipeline for autonomous drone tracking and navigation using ArUco markers in a warehouse environment. It integrates:
- **BCR Bot** (ground robot) for navigation via Nav2
- **SJTU Drone** (quadcopter) for aerial inspection and marker detection
- **ArUco detection & path generation** for building a “snake” trajectory over all markers
- **Automatic drone flight** along the generated trajectory (PID‐based)
- **Automatic landing** using bottom‐facing ArUco detection

Below is a step‐by‐step guide to recreate the workspace, build all packages, and launch each component in the proper order.

---

## Prerequisites

1. **Operating System**: Ubuntu 20.04 LTS (or compatible)  
2. **ROS 2 Distribution**: Foxy (or Galactic/Humble)  
3. **Colcon**: `sudo apt install python3-colcon-common-extensions`  
4. **Gazebo** (matching your ROS 2 dists):  
   ```bash
   sudo apt install ros-<ros2-distro>-gazebo-ros-pkgs \
                       ros-<ros2-distro>-gazebo-ros2-control
   ```  
5. **Nav2 Stack**:  
   ```bash
   sudo apt install ros-<ros2-distro>-nav2-bringup \
                       ros-<ros2-distro>-nav2-amcl \
                       ros-<ros2-distro>-nav2-map-server \
                       ros-<ros2-distro>-nav2-lifecycle-manager
   ```  
6. **OpenCV + CvBridge**:  
   ```bash
   sudo apt install ros-<ros2-distro>-cv-bridge \
                       ros-<ros2-distro>-vision-opencv \
                       libopencv-dev python3-opencv
   ```  
7. **Python packages** (will be installed in the ROS 2 workspace when needed):  
   - `numpy`  
   - `pyyaml`  
   - `matplotlib`  
   - `transforms3d`  
   - `tkinter` (for the GUI windows)  
   - `Pillow` (for displaying images in Tkinter)  
   If any of these are missing, you can install via:  
   ```bash
   sudo apt install python3-numpy python3-yaml python3-matplotlib \
                    python3-tk python3-pil python3-transforms3d
   ```

---

## Repository Layout

Assume you have three packages/folders that need to be placed under `src/` in a single ROS 2 workspace:

1. **sjtu_drone_bringup**  
   - Contains `launch/mobile_robot_and_drone_bringup.launch.py`  
   - Responsible for spawning the BCR Bot and the SJTU Drone in Gazebo, plus teleop and RViz.

2. **bcr_bot** (sometimes called `bcr_bot-ros2`)  
   - Contains `launch/nav2.launch.py`  
   - Provides a ROS 2–compatible BCR Bot with URDF, maps, Nav2 configuration, and a remapper script.

3. **sjtu_drone_control**  
   - Contains all custom nodes:  
     - `rack_navigator.py`  
     - `aruco_full_pipeline.py`  
     - `drone_auto_mode.py`  
     - `land_drone_after_finish.py`  
   - Also has a `config/aruco_gazebo/` folder (where YAML outputs are stored).

Below is a sample directory tree:

```
warehouse_drone_ws/
└── src/
    ├── sjtu_drone_bringup/
    │   └── launch/
    │       └── mobile_robot_and_drone_bringup.launch.py
    ├── bcr_bot/
    │   ├── config/
    │   │   ├── bcr_map.yaml
    │   │   ├── nav2_params.yaml
    │   │   └── amcl_params.yaml
    │   ├── launch/
    │   │   └── nav2.launch.py
    │   └── remapper.py
    └── sjtu_drone_control/
        ├── config/
        │   └── aruco_gazebo/
        │       └── (YAML outputs will be written here)
        ├── rack_navigator.py
        ├── aruco_full_pipeline.py
        ├── drone_auto_mode.py
        └── land_drone_after_finish.py
```

---

## Step 1: Create and Build the ROS 2 Workspace

1. **Create a new workspace** (e.g. `~/warehouse_drone_ws`):

   ```bash
   mkdir -p ~/warehouse_drone_ws/src
   cd ~/warehouse_drone_ws/src
   ```

2. **Copy or clone** the three packages into `src/`:

   - If you have local folders already downloaded:
     ```bash
     cp -r /path/to/sjtu_drone_bringup       .
     cp -r /path/to/bcr_bot-ros2 bcr_bot      # rename it to “bcr_bot” for consistency
     cp -r /path/to/sjtu_drone_control       .
     ```
   - _Or_, if they’re hosted in Git repositories:
     ```bash
     git clone https://github.com/Phetzxc/Simulation_Drone_Warehouse.git       sjtu_drone_bringup
     git clone https://github.com/Phetzxc/Simulation_Drone_Warehouse.git             bcr_bot
     git clone https://github.com/Phetzxc/Simulation_Drone_Warehouse.git       sjtu_drone_control
     ```

3. **Build the workspace**:

   ```bash
   cd ~/warehouse_drone_ws
   colcon build --symlink-install
   ```

   - If any dependencies are missing (e.g. `transforms3d`, `tkinter`, etc.), install them with `sudo apt` or `pip3`, then rebuild.

4. **Source the setup script**:

   ```bash
   source ~/warehouse_drone_ws/install/setup.bash
   ```

---

## Step 2: Launch the Mobile Robot + Drone in Gazebo

In a new terminal (and **source** the workspace again):

```bash
cd ~/warehouse_drone_ws
source install/setup.bash
ros2 launch sjtu_drone_bringup mobile_robot_and_drone_bringup.launch.py
```

- **What happens here?**  
  1. Gazebo is launched with both the BCR Bot (ground robot) and the SJTU Drone.  
  2. A `robot_state_publisher` node publishes TF for the BCR Bot URDF.  
  3. The `spawn_entity.py` node spawns the BCR Bot model into Gazebo.  
  4. Teleoperation nodes (`teleop_joystick` or `teleop`) for the drone are started under the namespace `drone/`.  
  5. If you open RViz (launched by this file), you can visualize both robot and drone frames.

Leave this window open—Gazebo and RViz will stay running.

---

## Step 3: Launch Nav2 for the BCR Bot

In a second terminal (again, **source** your workspace):

```bash
cd ~/warehouse_drone_ws
source install/setup.bash
ros2 launch bcr_bot nav2.launch.py
```

- **What happens here?**  
  1. The Nav2 “bringup” launch file starts all Nav2 nodes:  
     - **`map_server`** (serves the `bcr_map.yaml` located in `bcr_bot/config/`),  
     - **`amcl`** (localization using the same map, with parameters from `amcl_params.yaml`),  
     - **TF static publisher** from `map → odom`,  
     - **`remapper.py`** (if any topic remapping is required by your robot’s configuration),  
     - and also launches an RViz instance preconfigured to show the Nav2 view.  
  2. You should see your BCR Bot model in RViz on the top‐down map.

Nav2 will wait for navigation goals over the `/navigate_to_pose` action server.

---

## Step 4: Navigate to a Rack & Take Off Drone

In a third terminal (still source the workspace):

```bash
cd ~/warehouse_drone_ws
source install/setup.bash
ros2 run sjtu_drone_control rack_navigator
```

- **What this node does (`rack_navigator.py`):**  
  1. Prompts you:  
     ```
     What's rack?
     - rack_A
     - rack_B
     >> 
     ```  
  2. If you type `rack_A` or `rack_B`, it looks up the corresponding goal pose from `RACK_GOALS`.  
  3. It creates a `NavigateToPose` action client on `/navigate_to_pose` → sends the goal.  
  4. When the robot arrives at that rack (Nav2 reports “succeeded”), it publishes a `std_msgs/Empty` message to `/simple_drone/takeoff`.  
     - **Result**: The drone—already spawned by the first launch—receives a takeoff command and ascends to a preset altitude (handled by your drone’s autopilot/driver).

---

## Step 5: Run ArUco Detection & Path Generation

In a fourth terminal (source the workspace):

```bash
cd ~/warehouse_drone_ws
source install/setup.bash
ros2 run sjtu_drone_control aruco_full_pipeline
```

- **What this node does (`aruco_full_pipeline.py`):**  
  1. Subscribes to `/simple_drone/front/image_raw` (the front camera feed from the drone).  
  2. Detects **6×6 ArUco markers** in real time (with a marker length of 0.08 m).  
  3. For each detected marker, it:  
     - Estimates the pose in the camera frame, converts to the drone’s “base” frame (`T_cam_to_base @ T_cam_to_marker`).  
     - Publishes a `PoseStamped` to `/base_to_aruco`.  
     - Stores detections → calculates average position/orientation per marker.  
  4. On Ctrl+C:  
     - Saves raw observations, averaged observations, and an offset “snake path” (with Z/Y snake ordering) to:  
       - `saw_marker_from_drone.yaml`  
       - `estimate_marker_from_drone.yaml`  
       - `gen_path_modified.yaml`  
     - Writes a comparison plot `compare_path.jpg`.

---

## Step 6: (Optional) Generate Quintic‐Trajectory YAML

If you need a time‐parameterized file for `drone_auto_mode`, generate `gen_trajectory_output.yaml`:

```bash
python3 generate_quintic_trajectory.py     --input config/aruco_gazebo/gen_path_modified.yaml     --output config/aruco_gazebo/gen_trajectory_output.yaml
```

---

## Step 7: Run Automatic Drone Flight

```bash
cd ~/warehouse_drone_ws
source install/setup.bash
ros2 run sjtu_drone_control drone_auto_mode
```

---

## Step 8: Automatic Landing

```bash
cd ~/warehouse_drone_ws
source install/setup.bash
ros2 run sjtu_drone_control land_drone_after_finish
```

---

## One‐Shot Command Summary

```bash
# Source workspace
cd ~/warehouse_drone_ws && source install/setup.bash

# Run Gazebo (TERM 1)
ros2 launch sjtu_drone_bringup mobile_robot_and_drone_bringup.launch.py
# Run Nav2   (TERM 2)
ros2 launch bcr_bot nav2.launch.py
# Navigate & takeoff (TERM 3)
ros2 run sjtu_drone_control rack_navigator
# Scan markers (TERM 4)
ros2 run sjtu_drone_control aruco_full_pipeline
# (Optional) generate quintic trajectory between scan & flight …
# Fly drone   (TERM 5)
ros2 run sjtu_drone_control drone_auto_mode
# Land drone  (TERM 6)
ros2 run sjtu_drone_control land_drone_after_finish
```

---

## Tips

- Ensure correct TF frames (`map→odom→base_link→camera_link`).
- Adjust marker sizes (`0.08 m` front, `0.30 m` bottom) if you use different sizes.
- Tune PID gains in `drone_auto_mode.py` & `land_drone_after_finish.py` to match your drone dynamics.
- Always Ctrl+C **aruco_full_pipeline.py** _after_ scanning **all racks** → it writes the needed YAML files.

