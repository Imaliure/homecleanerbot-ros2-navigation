# HomeCleanerBot – ROS2 Autonomous Navigation

This repository contains a **ROS2 Humble** based autonomous home cleaning robot simulation.
The project demonstrates **SLAM**, **Nav2 navigation**, and **goal-based autonomous movement**
inside a Gazebo-simulated indoor environment.

---

## 📌 Project Overview

- **Robot Type:** Differential drive indoor service robot  
- **Simulation:** Gazebo  
- **ROS Version:** ROS2 Humble  
- **Navigation Stack:** Nav2  
- **Localization:** AMCL  
- **Mapping:** SLAM Toolbox  
- **Visualization:** RViz2  
- **Deployment:** Docker

The robot is manually localized using **2D Pose Estimate** in RViz and navigated by sending goals
with **Publish Point / Nav2 Goal**.

---

## 🏠 Environment

- 2+1 apartment layout (living room, kitchen, bathroom, rooms)
- Walls, furniture, and obstacles modeled in Gazebo
- Suitable for SLAM and autonomous navigation testing

---

## 🤖 Robot Features

- LiDAR-based perception
- Differential drive kinematics
- Real-time obstacle avoidance
- Global + local costmaps
- Regulated Pure Pursuit controller

---

## 🧭 Navigation Workflow

1. Start Gazebo with the house world and robot
2. Launch Nav2 with a pre-built map
3. Open RViz2
4. Set initial pose using **2D Pose Estimate**
5. Send navigation goals using **Publish Point**
6. Robot plans and follows a path autonomously

---

## 🐳 Docker Setup

The project runs fully inside a Docker container using **osrf/ros:humble-desktop-full**.

### Build Image
```bash
docker build -t homecleanerbot:humble .
```

### Run Container
```bash
xhost +local:docker
docker run -it --name homecleanerbot_container \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  homecleanerbot:humble
```

---

## ▶️ Run Commands (Multiple Terminals)

### Terminal 1 – Gazebo
```bash
docker exec -it homecleanerbot_container bash
source /opt/ros/humble/setup.bash
cd /homecleanerbot_ws
source install/setup.bash
ros2 launch homecleaner_description gazebo.launch.py
```

### Terminal 2 – Navigation
```bash
docker exec -it homecleanerbot_container bash
source /opt/ros/humble/setup.bash
cd /homecleanerbot_ws
source install/setup.bash
ros2 launch homecleaner_description navigation.launch.py
```

### Terminal 3 – RViz
```bash
docker exec -it homecleanerbot_container bash
source /opt/ros/humble/setup.bash
ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

### Terminal 4 – Mission Script
```bash
docker exec -it homecleanerbot_container bash
source /opt/ros/humble/setup.bash
cd /homecleanerbot_ws
source install/setup.bash
python3 src/homecleaner_description/scripts/cleanup_mission.py
```

---

## 📊 Experimental Results

- Successful localization with AMCL
- Stable path planning and execution
- Robot reaches multiple target goals without collision
- Recovery behaviors triggered when needed

---

---

## 👥 Team Members & Task Distribution

| Member | Responsibility |
|------|---------------|
| Cihan Kurtbey | Gazebo world & robot modeling & Nav2 configuration |
| Ali Ure | SLAM & Docker setup & testing|

---

## 📄 License

This project is for **academic and educational purposes**.
