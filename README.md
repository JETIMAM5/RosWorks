# ROS Mobile Robot Controller

A ROS package for controlling a differential-drive robot (tested on TurtleBot3)  
with trajectory tracking, waypoint navigation, and future autonomous features.

---

## 📖 Overview

This repository contains a **Python-based ROS node** for controlling a mobile robot.

It currently supports:

- Moving straight with heading correction  
- Following polygonal trajectories (triangle, square, etc.)  
- Drawing circular trajectories  
- Navigating to user-defined goal coordinates

> The project is designed as a foundation for learning **robotics algorithms** such as obstacle avoidance, mapping, SLAM, path planning, and localization.
> Files in the *scripts* are not up to date . Planning to update them.

---

## 🚀 Features

- [x] Move straight while maintaining heading  
- [x] Polygon-based trajectories (triangle, square, etc.)  
- [x] Circular trajectory following  
- [x] Go-to-goal navigation with simple P-controller  
- [x] Obstacle avoidance (BUG0 Applied) *(BUG1,2,TANGENT are planned)* 
- [ ] Mapping & SLAM *(planned)*  
- [ ] Path planning *(planned)*  
- [ ] Localization *(planned)*  
- [ ] Full autonomous navigation *(planned)*

---

## 🤖 Roadmap

| Feature                  | Status | Notes        |
|--------------------------|--------|--------------|
| Basic trajectory tracking | ✅     | Implemented  |
| Go-to-goal navigation     | ✅     | P-controller |
| Obstacle avoidance        | ✅     | BUG0         |
| Mapping & SLAM            | 🚧     | Planned      |
| Path planning             | 🚧     | Planned      |
| Localization              | 🚧     | Planned      |
| Autonomous behaviors      | 🚧     | Planned      |

---

## 🧠 Background & Goals

This project aims to serve as a **learning playground for robotics**, combining:

- ROS1 fundamentals (nodes, topics, launch files)  
- Motion control & PID tuning  
- Autonomous navigation stack (obstacle avoidance, SLAM, localization)  
- Path planning algorithms (A\*, D\*, Bug, potential fields)  
- Integration with sensors (Lidar, IMU, Odometry)

By expanding this repository, this can evolve into a complete mobile robot platform.

---

Maintained by **[JETIMAM]** — Robotics enthusiast working with ROS, embedded systems, and autonomous mobile robots.
