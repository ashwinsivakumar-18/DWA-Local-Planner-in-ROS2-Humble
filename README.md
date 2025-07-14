# DWA Local Planner for TurtleBot3 Burger

## Overview
A ROS2 Humble package implementing a Dynamic Window Approach (DWA) local planner optimized for TurtleBot3 Burger. Features carefully tuned parameters for stable navigation and obstacle avoidance.

## Installation

  ```bash
    # 1. Create workspace
    mkdir -p ~/dwa_ws/src
    cd ~/dwa_ws/src
    
    # 2. Clone repository
    git clone https://github.com/ashwinsivakumar-18/DWA-Local-Planner-in-ROS2-Humble.git
    
    # 3. Build package
    cd ~/dwa_ws
    rosdep install -i --from-path src --rosdistro humble -y
    colcon build --symlink-install
    
    # 4. Setup environment
    echo "source ~/dwa_ws/install/setup.bash" >> ~/.bashrc
    echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
    source ~/.bashrc
```
## ⚙️ Configuration Parameters

### 🚗 Motion Constraints

| Parameter              | Default     | Description                          |
|------------------------|-------------|--------------------------------------|
| `max_speed`            | 0.15 m/s    | Maximum linear velocity              |
| `min_speed`            | 0.05 m/s    | Minimum forward velocity             |
| `max_turn_rate`        | 2.84 rad/s  | Maximum angular velocity (~163°/s)  |
| `max_acceleration`     | 0.1 m/s²    | Linear acceleration limit            |
| `max_deceleration`     | 0.2 m/s²    | Linear deceleration limit            |
| `max_turn_acceleration`| 1.0 rad/s²  | Angular acceleration limit           |

---

### 🧭 Navigation Parameters

| Parameter         | Default   | Description                        |
|-------------------|-----------|------------------------------------|
| `robot_radius`    | 0.12 m    | Physical robot radius              |
| `safety_margin`   | 0.15 m    | Additional obstacle clearance      |
| `goal_tolerance`  | 0.35 m    | Goal acceptance radius             |
| `slow_down_distance` | 0.75 m | Distance to begin deceleration     |
| `sim_samples`     | 15        | Velocity samples for simulation    |
| `eval_samples`    | 80        | Trajectories to evaluate           |

---

### 🧮 Cost Function Weights

| Parameter         | Default | Purpose                         |
|-------------------|---------|----------------------------------|
| `goal_weight`     | 3.0     | Goal attraction strength         |
| `obstacle_weight` | 3.0     | Obstacle avoidance strength      |
| `heading_weight`  | 1.5     | Path alignment importance        |

## 🧠 How the System Works (Multi-Terminal Execution)

To run the full custom DWA planner system with Gazebo simulation and RViz2 visualization, use the following three terminals:

---

### ✅ Terminal 1 – Launch Gazebo Simulation

```bash
ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage4.launch.py
```
Launches the TurtleBot3 simulation in Gazebo (stage4 environment).

Spawns the robot with sensors and physics.

Publishes essential topics like:

    /odom – robot odometry

    /scan – laser scan

    /tf – coordinate transforms


### ✅ Terminal 2 - Start DWA planner
```bash
ros2 launch dwa_planner planner.launch.py
ros2 run custom_dwa_planner_cpp dwa_planner --ros-args -p goal_x:=1.0 -p goal_y:=1.5
```

  

    planner.launch.py sets up required parameters/nodes.

    dwa_planner is the custom local planner:

        Subscribes to /odom, /scan

        Publishes velocity commands to /cmd_vel

        Computes safe and goal-directed motion in real-time

        Parameters:

            goal_x / goal_y – target position


### ✅ Terminal 3 - Launch RViz visualization
```bash
ros2 launch turtlebot3_bringup rviz2.launch.py 
```
      Launches RViz2 with the default TurtleBot3 config.

    Visualizes:

        Robot model and position

        Laser scan data

        TF tree

        Planned path and goals

🖼️ Add Markers in RViz2

To visualize the DWA planner’s evaluated trajectories or selected path:

    Open RViz2.

    In the "Displays" panel, click Add.

    Choose MarkerArray.


---

## ✅ Project Status

This repository implements a **custom Dynamic Window Approach (DWA) local planner** for TurtleBot3 in **ROS2 Humble**, built entirely from scratch without relying on `nav2_dwb_controller`.  
Simulation is run in **Gazebo**, and the planner includes visualization using `MarkerArray` in **RViz2**.  
The planner is tested in `stage4` environments and supports real-time navigation to user-defined goals using velocity sampling and cost function optimization.

---

## 🤝 Contributing

Contributions are welcome! 🚀  
If you'd like to improve the planner, add features, fix bugs, or optimize performance, follow these steps:

1. Fork the repository
2. Create a new branch (`git checkout -b feature-name`)
3. Commit your changes (`git commit -m "Add feature"`)
4. Push to the branch (`git push origin feature-name`)
5. Open a **Pull Request**

📌 Issues and suggestions are also welcome via the **Issues** tab.

---

## 📄 License

This project is licensed under the **MIT License**.




