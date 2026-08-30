````md
# F1TENTH Gym ROS 2 — Docker Workflow

## Setup

- Host OS: Ubuntu 24.04
- Host ROS 2: Jazzy
- Docker ROS 2: Foxy
- Workspace: `/home/ms/sim_ws`
- Docker workspace: `/sim_ws`

---

# 🛑 EMERGENCY STOP

Run this **inside the Foxy Docker container**:

```bash
ros2 topic pub --once /drive \
  ackermann_msgs/msg/AckermannDriveStamped \
  "{drive: {speed: 0.0, steering_angle: 0.0}}"
````

---

# TAB 1 — LAUNCH DOCKER + F1TENTH GYM

## Ubuntu HOST

**IMPORTANT: Start from `/home/ms/sim_ws`, NOT `/home/ms/sim_ws/src/f1tenth_gym_ros`.**

```bash
cd /home/ms/sim_ws
```

Launch the Foxy Docker container:

```bash
sudo rocker --x11 \
  --env LIBGL_ALWAYS_SOFTWARE=1 \
  --env GALLIUM_DRIVER=llvmpipe \
  --volume "$PWD:/sim_ws" \
  -- f1tenth_gym_ros
```

---

## Inside Docker

```bash
cd /sim_ws

source /opt/ros/foxy/setup.bash
source /sim_ws/install/local_setup.bash
```

Launch F1TENTH:

```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

Keep this terminal running.

---

# TAB 2 — RUN MY CUSTOM AGENT

Open a new Ubuntu terminal.

Find the running container:

```bash
docker ps
```

Enter the same container:

```bash
docker exec -it CONTAINER_NAME bash
```

Inside Docker:

```bash
cd /sim_ws

source /opt/ros/foxy/setup.bash
source /sim_ws/install/local_setup.bash
```

## Build custom agents

Build from `/sim_ws`:

```bash
colcon build --packages-select ms_agents_pkg --symlink-install
```

Then source the build:

```bash
source /sim_ws/install/local_setup.bash
```

Check available agents:

```bash
ros2 pkg executables ms_agents_pkg
```

Run the simple LiDAR avoider:

```bash
ros2 run ms_agents_pkg simple_lidar_avoider
```

Or run the basic drive agent:

```bash
ros2 run ms_agents_pkg drive_agent
```

---

# TAB 3 — RUN GIU'S GAP FOLLOWER

Open another Ubuntu terminal.

Enter the existing container:

```bash
docker ps
```

```bash
docker exec -it CONTAINER_NAME bash
```

Inside Docker:

```bash
source /opt/ros/foxy/setup.bash
source /sim_ws/install/local_setup.bash
```

Run Giu's steering/speed controller:

```bash
python3 /sim_ws/src/f1tenth_gym_ros/references/giu_gap_follower/gap_follower/steering_speed_control.py \
  --ros-args \
  -p drive_topic:=/drive \
  -p control_selector_topic:=/control_selector
```

Keep this terminal running.

---

# TAB 4 — SELECT GIU'S GAP FOLLOWER

Enter the same container:

```bash
docker exec -it CONTAINER_NAME bash
```

Source Foxy:

```bash
source /opt/ros/foxy/setup.bash
source /sim_ws/install/local_setup.bash
```

Select gap following:

```bash
ros2 topic pub --rate 2 /control_selector \
  std_msgs/msg/String \
  "{data: gap_following}"
```

Keep this command running.

---

# EDITING / DEVELOPING AGENTS

Edit files on the Ubuntu host using VS Code:

```text
/home/ms/sim_ws/src/ms_agents_pkg/
```

For example:

```text
/home/ms/sim_ws/src/ms_agents_pkg/ms_agents_pkg/drive_agent.py
/home/ms/sim_ws/src/ms_agents_pkg/ms_agents_pkg/simple_lidar_avoider.py
```

Inside Docker, the exact same files are:

```text
/sim_ws/src/ms_agents_pkg/ms_agents_pkg/drive_agent.py
/sim_ws/src/ms_agents_pkg/ms_agents_pkg/simple_lidar_avoider.py
```

After editing:

```bash
cd /sim_ws

colcon build --packages-select ms_agents_pkg --symlink-install

source install/local_setup.bash
```

Then run the agent:

```bash
ros2 run ms_agents_pkg simple_lidar_avoider
```

---

# ADDING A NEW AGENT

Create:

```text
/home/ms/sim_ws/src/ms_agents_pkg/ms_agents_pkg/my_new_agent.py
```

Add its executable to `setup.py`:

```python
entry_points={
    'console_scripts': [
        'drive_agent = ms_agents_pkg.drive_agent:main',
        'simple_lidar_avoider = ms_agents_pkg.simple_lidar_avoider:main',
        'my_new_agent = ms_agents_pkg.my_new_agent:main',
    ],
},
```

Then inside Docker:

```bash
cd /sim_ws

colcon build --packages-select ms_agents_pkg --symlink-install

source install/local_setup.bash
```

Run:

```bash
ros2 run ms_agents_pkg my_new_agent
```

---

# USEFUL COMMANDS

## Check running Docker container

```bash
docker ps
```

## Enter running container

```bash
docker exec -it CONTAINER_NAME bash
```

## Check ROS packages

```bash
ros2 pkg list | grep -E "f1tenth|ms_agents"
```

Expected:

```text
f1tenth_gym_ros
ms_agents_pkg
```

## Check custom executables

```bash
ros2 pkg executables ms_agents_pkg
```

Expected:

```text
ms_agents_pkg drive_agent
ms_agents_pkg simple_lidar_avoider
```

## Check topics

```bash
ros2 topic list
```

## Check LiDAR

```bash
ros2 topic hz /scan
```

## Check drive commands

```bash
ros2 topic echo /drive
```

## Check drive connections

```bash
ros2 topic info /drive
```

---

# WORKSPACE STRUCTURE

Host:

```text
/home/ms/sim_ws/
└── src/
    ├── f1tenth_gym_ros/
    │   └── references/
    │       └── giu_gap_follower/
    │
    └── ms_agents_pkg/
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── resource/
        │   └── ms_agents_pkg
        └── ms_agents_pkg/
            ├── __init__.py
            ├── drive_agent.py
            └── simple_lidar_avoider.py
```

Docker:

```text
/sim_ws/
└── src/
    ├── f1tenth_gym_ros/
    └── ms_agents_pkg/
```

The entire workspace is mounted:

```text
/home/ms/sim_ws  →  /sim_ws
```

---

# IMPORTANT RULES

## 1. Always launch Docker from the workspace root

Correct:

```bash
cd /home/ms/sim_ws
```

Then:

```bash
--volume "$PWD:/sim_ws"
```

Do NOT do:

```bash
cd /home/ms/sim_ws/src/f1tenth_gym_ros
```

because that would mount `f1tenth_gym_ros` itself as `/sim_ws` and the workspace would be wrong.

---

## 2. Build inside Docker

Always build from:

```bash
cd /sim_ws
```

Use:

```bash
colcon build --packages-select ms_agents_pkg --symlink-install
```

Do NOT build the Foxy workspace using host ROS 2 Jazzy.

---

## 3. Source Foxy in every new Docker terminal

```bash
source /opt/ros/foxy/setup.bash
source /sim_ws/install/local_setup.bash
```

---

## 4. All F1TENTH ROS commands run inside Docker

The Ubuntu host uses ROS 2 Jazzy.

F1TENTH uses ROS 2 Foxy.

Therefore:

```text
Ubuntu 24.04
    │
    ├── ROS 2 Jazzy
    │
    └── Docker
         │
         └── ROS 2 Foxy
              │
              └── /sim_ws
                   ├── f1tenth_gym_ros
                   └── ms_agents_pkg
```

Use Docker for:

* `ros2 launch`
* `ros2 run`
* `ros2 topic`
* `colcon build`
* F1TENTH agents
* F1TENTH debugging

---

# NORMAL WORK SESSION

### Terminal 1

```bash
cd /home/ms/sim_ws

sudo rocker --x11 \
  --env LIBGL_ALWAYS_SOFTWARE=1 \
  --env GALLIUM_DRIVER=llvmpipe \
  --volume "$PWD:/sim_ws" \
  -- f1tenth_gym_ros
```

Inside:

```bash
source /opt/ros/foxy/setup.bash
source /sim_ws/install/local_setup.bash

ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

### Terminal 2

```bash
docker ps
docker exec -it CONTAINER_NAME bash

source /opt/ros/foxy/setup.bash
source /sim_ws/install/local_setup.bash

cd /sim_ws
colcon build --packages-select ms_agents_pkg --symlink-install
source install/local_setup.bash

ros2 run ms_agents_pkg simple_lidar_avoider
```

### Terminal 3 — Giu's agent instead

```bash
docker exec -it CONTAINER_NAME bash

source /opt/ros/foxy/setup.bash
source /sim_ws/install/local_setup.bash

python3 /sim_ws/src/f1tenth_gym_ros/references/giu_gap_follower/gap_follower/steering_speed_control.py \
  --ros-args \
  -p drive_topic:=/drive \
  -p control_selector_topic:=/control_selector
```

### Terminal 4 — Giu's controller selection

```bash
docker exec -it CONTAINER_NAME bash

source /opt/ros/foxy/setup.bash
source /sim_ws/install/local_setup.bash

ros2 topic pub --rate 2 /control_selector \
  std_msgs/msg/String \
  "{data: gap_following}"
```

```
```
