# Warehouse Robot Simulation 🤖📦

A complete ROS 2 Jazzy and Gazebo Sim warehouse navigation and simulation environment for a differential drive robot ("Articubot"). This project features fully configured Lidar and Camera sensors, keyboard teleoperation, and an autonomous navigation stack using Nav2 and SLAM Toolbox.

---

## 🛠️ System Requirements
- **OS**: Ubuntu 24.04 (Noble)
- **ROS 2 Version**: Jazzy Jalisco
- **Simulator**: Gazebo Sim (Harmonic)

## 📦 Dependencies
Make sure you have the following ROS 2 packages installed:
```bash
sudo apt update
sudo apt install ros-jazzy-ros-gz \
                 ros-jazzy-xacro \
                 ros-jazzy-ros2-control \
                 ros-jazzy-ros2-controllers \
                 ros-jazzy-gz-ros2-control \
                 ros-jazzy-navigation2 \
                 ros-jazzy-nav2-bringup \
                 ros-jazzy-slam-toolbox \
                 ros-jazzy-teleop-twist-keyboard
```

*(Note: If you plan to use keyboard teleoperation, ensure you are running in an environment that supports GUI terminal emulators like `gnome-terminal` or `xterm`)*

---

## 🚀 Quick Start Guide

### 1. Build the Workspace
Navigate to the root of your workspace and build the package:
```bash
colcon build --packages-select warehouse_env
source install/setup.bash
```

### 2. Launch the Main Simulation (Terminal 1)
This will launch Gazebo Sim, spawn the warehouse and robot, bridge all sensor data to ROS 2, start the controllers, and open RViz.
```bash
source install/setup.bash
ros2 launch warehouse_env warehouse.launch.py
```

### 3. Drive the Robot Manually (Terminal 2)
To explore the warehouse manually, use the teleoperation launch file. A new terminal window will pop up with **WASD** controls.
```bash
source install/setup.bash
ros2 launch warehouse_env teleop.launch.py
```
* **Keys:** `W` (Forward), `S` (Backward), `A` (Rotate Left), `D` (Rotate Right)
* **Speed:** `Q` (Faster), `E` (Slower), `Space` (Stop)

---

## 🏎️ Holonomic vs. Non-Holonomic Mode

Understanding the movement constraints of your robot is crucial for navigation:

| Feature | Non-Holonomic (Your Robot) | Holonomic Mode |
| :--- | :--- | :--- |
| **Movement** | Differential Drive (Wheels) | Mecanum / Omni wheels |
| **Strafing** | **No**. Cannot move sideways. | **Yes**. Can move sideways/diagonally. |
| **Turning** | Must rotate to change direction. | Can move and rotate simultaneously. |
| **DOF** | 2 Controllable DOF (Linear X, Angular Z). | 3 Controllable DOF (Linear X, Linear Y, Angular Z). |

> [!NOTE]
> Your robot is **Non-Holonomic**. This means it has a "turning radius" and cannot instantly move left or right. The navigation stack (`Nav2`) is configured to account for these constraints.

---

### 4. Autonomous Navigation (Terminal 3)
If you want the robot to map the environment and drive itself, launch the Nav2 stack (which includes SLAM Toolbox):
```bash
source install/setup.bash
ros2 launch warehouse_env nav2.launch.py
```
Once launched, switch to your **RViz** window, click the **"2D Goal Pose"** tool in the top toolbar, and click/drag anywhere on the generated map to command the robot to drive there autonomously!

---

## 🧠 Custom Nodes & Scripts

This repository includes several custom Python scripts in the `scripts/` directory to seamlessly bridge Gazebo Sim and ROS 2 Jazzy:

* **`scan_frame_relay.py`**: Intercepts Lidar `/scan` data from Gazebo (which uses namespaced frames like `articubot/base_link/laser`) and rewrites the `frame_id` to `laser_frame` to perfectly align with the URDF TF tree.
* **`twist_stamper.py`**: Converts raw `Twist` messages from the teleop keyboard into `TwistStamped` messages. This is strictly required by the modern Jazzy `diff_drive_controller`.
* **`proximity_monitor.py`**: A utility node that processes Lidar data and provides real-time, color-coded terminal alerts for obstacles in four cardinal directions based on distance zones (Danger, Warning, Caution, Clear).

## 🎥 Visualization
The `warehouse.rviz` configuration is pre-loaded to visualize:
- The Robot Model (`/robot_description`)
- The active TF tree
- Live LaserScan data (Red outline)
- Live Camera feed (`/camera/image_raw`)
- Global Maps and Local Costmaps (when Nav2 is running)
- Global and Local planned paths
