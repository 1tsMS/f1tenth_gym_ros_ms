# F1TENTH Autonomous Racing Stack (ROS 2)

An end-to-end autonomous racing development stack built in ROS 2 Foxy using the F1TENTH Gym simulator.

This project goes from basic reactive safety nodes up to global trajectory optimization and two-car head-to-head racing with dynamic overtaking.

---

## Project Structure

```text
├── f1tenth notes.pdf           # Personal handwritten notes and math derivations
├── videos/                     # Screen recordings of all agents running in simulation
├── ms_agents_pkg/              # Custom ROS 2 package containing all control nodes
│   ├── ms_agents_pkg/
│   │   ├── safety_braking_agent.py   # Level 1: Time-to-Collision (TTC) emergency braking
│   │   ├── pid_wall_follower.py      # Level 2: 2-beam PID wall follower
│   │   ├── gap_follower.py           # Level 3: Reactive Follow-The-Gap local planner
│   │   ├── waypoint_logger.py        # Waypoint recording tool (saves x, y, v to CSV)
│   │   ├── keyboard_teleop.py        # Custom non-blocking terminal driving node
│   │   ├── smooth_waypoints.py       # Cubic spline smoothing and speed profiler
│   │   ├── pure_pursuit.py           # Level 4: Pure pursuit trajectory tracker
│   │   ├── dynamic_overtaker.py      # Level 5: Multi-agent overtaking state machine
│   │   └── reset_cars.py             # Utility to teleport cars back to grid positions
│   ├── waypoints.csv                 # Optimized raceline waypoints for Levine circuit
│   ├── package.xml
│   └── setup.py
└── f1tenth_gym_ros/            # Simulator bridge package and launch files
    ├── config/
    │   ├── sim.yaml                  # 1-car simulation configuration
    │   └── sim_two_agents.yaml       # 2-car simulation configuration
    └── launch/
        ├── gym_bridge_launch.py      # Launch file for solo testing (1 car)
        └── two_car_launch.py         # Launch file for head-to-head racing (2 cars)
```

---

## Overview of Algorithms

### 1. Automatic Emergency Braking (AEB)
- File: `ms_agents_pkg/safety_braking_agent.py`
- What it does: Calculates Instantaneous Time-to-Collision (iTTC) for every LiDAR beam using the vehicle speed. If iTTC drops below 0.4 seconds inside the direct forward path, it overrides the drive command and commands maximum braking to prevent high-speed crashes.

### 2. PID Wall Follower
- File: `ms_agents_pkg/pid_wall_follower.py`
- What it does: Uses 2 LiDAR beams (at 45 degrees and 90 degrees) to project the orientation angle of the wall and estimate future distance. Runs a PD controller to steer the car and maintain a fixed distance from the left or right wall.

### 3. Follow The Gap (FTG)
- File: `ms_agents_pkg/gap_follower.py`
- What it does: A reactive local planner that finds the closest obstacle in the LiDAR scan, draws a safety bubble around it to eliminate tight gaps, finds the deepest open corridor ahead, and steers toward its center with low-pass steering smoothing.

### 4. Raceline Generation and Pure Pursuit
- Files: `ms_agents_pkg/waypoint_logger.py`, `ms_agents_pkg/smooth_waypoints.py`, `ms_agents_pkg/pure_pursuit.py`
- What it does:
  - Waypoints are recorded and smoothed using periodic cubic splines.
  - Speeds along the path are calculated based on local curvature: $v = \sqrt{a_{lat} / \kappa}$.
  - The Pure Pursuit node tracks the $(x, y)$ path using a dynamic lookahead distance scaled by speed, plus anticipatory braking before corner entries.

### 5. Multi-Agent Dynamic Overtaker
- File: `ms_agents_pkg/dynamic_overtaker.py`
- What it does: A hierarchical state machine for head-to-head racing. It follows the optimal raceline by default. When it detects a slower car ahead (under 2.2 meters), it shifts laterally into the open passing lane, boosts speed to 4.0 m/s, holds the pass steadily, and merges back onto the raceline once clear.

---

## How to Build and Run

### Build the packages
```bash
cd /sim_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select f1tenth_gym_ros ms_agents_pkg --symlink-install
source /sim_ws/install/local_setup.bash
```

---

### Solo Car Simulation (Levels 1 to 4)

1. Launch the simulator (1 car):
```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

2. Run any controller in a second terminal:
```bash
# Level 1: Safety Braking Failsafe
ros2 run ms_agents_pkg safety_braking_agent

# Level 2: PID Wall Follower
ros2 run ms_agents_pkg pid_wall_follower

# Level 3: Follow The Gap
ros2 run ms_agents_pkg gap_follower

# Level 4: Pure Pursuit
ros2 run ms_agents_pkg pure_pursuit
```

---

### Two-Car Racing Simulation (Level 5)

1. Launch the simulator (2 cars):
```bash
ros2 launch f1tenth_gym_ros two_car_launch.py
```

2. Run Car 1 (Lead Car, running PID wall follower or gap follower):
```bash
ros2 run ms_agents_pkg pid_wall_follower
```

3. Run Car 2 (Chasing Car, running Dynamic Overtaker):
```bash
ros2 run ms_agents_pkg dynamic_overtaker
```

4. Reset both cars back to the starting line at any time:
```bash
ros2 run ms_agents_pkg reset_cars
```

---

## Demos and Notes

- `f1tenth notes.pdf`: My handwritten notes.
- `videos/`: Screen recordings showing the performance of each algorithm in RViz.

