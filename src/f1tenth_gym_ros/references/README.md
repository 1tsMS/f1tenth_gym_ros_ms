# External Agent References

## GIU-F1Tenth gap_follower

Source: https://github.com/GIU-F1Tenth/gap_follower

This exact ROS 2 package is included at `references/giu_gap_follower`. It is an MIT-licensed F1TENTH gap-following implementation with LiDAR filtering, obstacle edge expansion, adaptive speed control, and emergency stopping.

The source is kept unchanged. It was tested by its authors with ROS 2 Humble/Iron, so treat it as a learning reference rather than a guarantee of Foxy/Jazzy compatibility.

For this repository, its default topics need remapping:

- Input: `/scan`
- Upstream output: `/ackermann_cmd`
- This bridge output: `/drive`

Run it against one car by setting `num_agent: 1` in `config/sim.yaml`, rebuilding, and launching the simulator. Then, in another container shell:

```bash
source /opt/ros/foxy/setup.bash
source /sim_ws/install/local_setup.bash
python3 /sim_ws/src/f1tenth_gym_ros/references/giu_gap_follower/gap_follower/steering_speed_control.py --ros-args \
  -p drive_topic:=/drive \
  -p control_selector_topic:=/control_selector
```

Activate the algorithm in a third shell:

```bash
source /opt/ros/foxy/setup.bash
ros2 topic pub --once /control_selector std_msgs/msg/String \
  "{data: gap_following}"
```

With `num_agent: 2`, the bridge also requires a command stream on `/opp_drive`; run a separate opponent controller before expecting the simulation to advance.
