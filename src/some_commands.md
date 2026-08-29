// STOPP
ros2 topic pub --once /drive \
  ackermann_msgs/msg/AckermannDriveStamped \
  "{drive: {speed: 0.0, steering_angle: 0.0}}"

**TAB 1**

cd /home/ms/sim_ws/src/f1tenth_gym_ros

sudo rocker --x11 \
  --env LIBGL_ALWAYS_SOFTWARE=1 \
  --env GALLIUM_DRIVER=llvmpipe \
  --volume "$PWD:/sim_ws" \
  -- f1tenth_gym_ros

source /opt/ros/foxy/setup.bash
source /sim_ws/install/local_setup.bash

ros2 launch f1tenth_gym_ros gym_bridge_launch.py


**TAB 2**

docker ps

docker exec -it CONTAINER_NAME bash

source /opt/ros/foxy/setup.bash
source /sim_ws/install/local_setup.bash

python3 /sim_ws/src/f1tenth_gym_ros/references/giu_gap_follower/gap_follower/steering_speed_control.py \
  --ros-args \
  -p drive_topic:=/drive \
  -p control_selector_topic:=/control_selector


**TAB 3**

docker exec -it CONTAINER_NAME bash

source /opt/ros/foxy/setup.bash
source /sim_ws/install/local_setup.bash

ros2 topic pub --rate 2 /control_selector \
  std_msgs/msg/String \
  "{data: gap_following}"


**BUILD CUSTOM AGENTS**

cd /sim_ws

colcon build --packages-select ms_agents_pkg --symlink-install

source install/local_setup.bash