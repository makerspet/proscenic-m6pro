# Proscenic M6 Pro — ROS 2 robot description

ROS 2 robot description and configuration for the **Proscenic M6 Pro** robot vacuum,
integrated with the [Kaia.ai](https://kaia.ai) software [platform](https://github.com/kaiaai/).

Unlike the ESP32 + [micro-ROS](https://micro.ros.org/) Maker's Pet robots, the Proscenic
M6 Pro is a stock commercial vacuum. It is bridged to ROS 2 by
[`remakeai/vacuum_ros2_bridge`](https://github.com/remakeai/vacuum_ros2_bridge), which talks
to the on-board **SangamIO** daemon (part of [VacuumTiger](https://github.com/codetiger/VacuumTiger))
over TCP port 5555. Setup is documented in this
[tutorial](https://makerspet.com/blog/tutorial-connect-robot-vacuum-cleaner-to-ros-2-proscenic-m6-pro/).

```
Proscenic M6 Pro (SangamIO, TCP 5555) <--Wi-Fi/LAN--> PC (vacuum_ros2_bridge) <--> ROS 2
```

## Package contents
- `urdf/` — xacro description of the ~349 mm round vacuum (body + LiDAR turret, diff-drive
  wheels, caster). Frames follow the Kaia.ai convention: `base_footprint → base_link → base_scan`.
- `config/ekf.yaml` — `robot_localization` EKF that fuses `/odom` + `/imu` and publishes the
  `odom → base_footprint` transform (the bridge publishes the `/odom` topic but not this TF,
  which cartographer requires).
- `config/cartographer_lds_2d.lua`, `config/navigation.yaml`, … — SLAM / Nav2 tuning.
- `config/gz_bridge.yaml`, `urdf/plugins.xacro` — Gazebo simulation (diff-drive, odometry,
  gpu_lidar) — simulation does **not** use the bridge.
- `launch/bringup.launch.py` — physical bring-up: bridge + `robot_state_publisher` + EKF.

## Usage

Select the robot model (used by the shared Kaia.ai launch files):
```
kaia config robot.model proscenic_m6pro
```

### Simulation (no robot needed)
```
ros2 launch kaiaai_gazebo world.launch.py
ros2 launch kaiaai_bringup navigation.launch.py use_sim_time:=true slam:=True
ros2 run kaiaai_teleop teleop_keyboard
```

### Physical robot
The robot must be on the LAN running SangamIO (see the tutorial for flashing/Wi-Fi).
```
ros2 launch proscenic_m6pro bringup.launch.py robot_ip:=<robot-ip>
ros2 launch kaiaai_bringup navigation.launch.py slam:=True
ros2 run kaiaai_teleop teleop_keyboard
ros2 run nav2_map_server map_saver_cli -f ~/maps/map
```

## Notes
- URDF dimensions are approximate (~349 mm diameter, ~95 mm height, 0.233 m wheel base to
  match the bridge's odometry). Refine against measurements of your robot.
- Vacuum-specific actuators (vacuum/brushes/water pump, LEDs) are exposed by the bridge via
  `/set_actuator`, `/set_led`, `/set_lidar` and the `/actuator_cmd`, `/led_cmd` topics.

## License
Apache 2.0
