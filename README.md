# AutonomousRobotNavigation

## Prerequisites

- **Windows 11** (fully updated)
- **WSL2** with Ubuntu 24.04
- **WSLg** (included in Windows 11 — provides GUI support)
- **Docker Desktop** for Windows with WSL2 backend enabled

### 1. Install WSL2 and Ubuntu 24.04

Open PowerShell as Administrator:

```powershell
wsl --install -d Ubuntu-24.04
```

Restart when prompted, then launch Ubuntu from the Start Menu and complete setup.

### 2. Install Docker Desktop

1. Download from <https://www.docker.com/products/docker-desktop>
2. During installation, ensure **WSL2 Backend** is selected.
3. After installation, go to **Settings → Resources → WSL Integration** and enable it for Ubuntu 24.04.
4. Verify from inside WSL:

```bash
docker run hello-world
```

### 3. Allow GUI access (run once per session)

```bash
sudo apt install -y x11-xserver-utils
xhost +local:docker
```

---

## Clone the Repository

```bash
git clone <url>
```

## Start the Docker Container

From the `ros_jazzy` folder:

```bash
./run.sh
```

All subsequent commands are run **inside the container**.

---

## Install ROS Dependencies if needed

Run once inside the container after cloning:

```bash
sudo apt update && sudo apt install -y \
    ros-jazzy-gazebo-msgs \
    ros-jazzy-turtlebot3-description \
    ros-jazzy-turtlebot3-simulations \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-interfaces
```

---

## TurtleBot3 Navigation & Mapping

### Terminal 1 — Launch Gazebo

```bash
cd /home/ros/ros_ws
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py use_sim_time:=True
```

### Terminal 2 — Build & Launch Navigation or Mapping

```bash
cd /home/ros/ros_ws
rm -rf build/my_bot_controller install/my_bot_controller
colcon build --symlink-install --packages-select my_bot_controller
source install/setup.bash
```

**Navigation:**
```bash
ros2 launch my_bot_controller navigation.launch.py use_sim_time:=True
```

**Mapping:**
```bash
ros2 launch my_bot_controller mapping.launch.py
```

### Terminal 3 — Visualiser

```bash
ros2 run my_bot_controller visualiser
```

---

## Radar Navigation & Mapping

### Terminal 1 — Build & Launch the Radar World

```bash
cd /home/ros/ros_ws
rm -rf build/my_bot_controller install/my_bot_controller
colcon build --symlink-install --packages-select my_bot_controller
source install/setup.bash
ros2 launch my_bot_controller radar_world.launch.py
```

### Terminal 2 — Navigation or Mapping

**Navigation:**
```bash
cd /home/ros/ros_ws
source install/setup.bash
ros2 launch my_bot_controller radar_navigation.launch.py
```

**Mapping:**
```bash
cd /home/ros/ros_ws
source install/setup.bash
ros2 launch my_bot_controller radar_mapping.launch.py
```

---

## Saving a Map

```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros_ws/src/my_bot_controller/maps/new_world_map
```
