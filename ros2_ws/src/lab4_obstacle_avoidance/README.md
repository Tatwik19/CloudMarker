# lab4_obstacle_avoidance

Reactive navigation package for RAS 598 Lab 4.

## What this package provides

- A ROS 2 node: `reactive_navigator`
- A service: `/toggle_navigation` (`std_srvs/srv/SetBool`) to enable/disable navigation
- A launch file: `turtlebot_bringup.launch.py`
- Behavior modes:
  - A: Random 90° to 180° turn when front obstacle is below `safety_distance`
  - B: 180° turn when both left and right are below `safety_distance`
  - C: 90° turn toward clearer side when front obstacle is below `choice_c_front_threshold`

## Final working architecture

- TurtleBot09 runs robot base/sensors and DDS discovery server.
- VM runs this package (`lab4_obstacle_avoidance`) and connects to robot DDS server.
- Both robot and VM use `ROS_DOMAIN_ID=9`.

## Build (VM)

```bash
cd /home/eva/ros2_ws
colcon build --packages-select lab4_obstacle_avoidance
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## Environment setup (required)

### On robot (`ubuntu@turtlebot09`)

```bash
source /opt/ros/jazzy/setup.bash
source /etc/turtlebot4/setup.bash 2>/dev/null || true
export ROS_DOMAIN_ID=9
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=192.168.50.209:11811
export ROS_SUPER_CLIENT=True
unset ROS_LOCALHOST_ONLY
```

### On VM (both terminals)

```bash
cd /home/eva/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=9
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=192.168.50.209:11811
export ROS_SUPER_CLIENT=True
unset ROS_LOCALHOST_ONLY
```

Then refresh daemon in each shell:

```bash
ros2 daemon stop
ros2 daemon start
```

## Launch flow (VM + Robot)

### Terminal R (robot SSH)

1. SSH to robot:

```bash
ssh ubuntu@turtlebot09
```

2. Apply robot env (above).
3. Start your normal robot bringup (Lab 3 command).
4. Verify robot publishes scan:

```bash
ros2 topic info /robot_09/scan
```

Expected: `Publisher count > 0`.

### Terminal A (VM)

1. Apply VM env (above).
2. Launch lab4 node:

```bash
ros2 launch lab4_obstacle_avoidance turtlebot_bringup.launch.py
```

### Terminal B (VM)

1. Apply VM env (above).
2. Enable navigation:

```bash
ros2 service call /toggle_navigation std_srvs/srv/SetBool "{data: true}"
```

Disable when needed:

```bash
ros2 service call /toggle_navigation std_srvs/srv/SetBool "{data: false}"
```

## Verification checklist

Run in VM Terminal B:

```bash
ros2 node list | grep reactive_navigator
ros2 service list | grep toggle_navigation
ros2 topic info /robot_09/scan
ros2 topic info /robot_09/cmd_vel_unstamped
ros2 topic echo /robot_09/cmd_vel_unstamped --once
```

Expected:

- `/reactive_navigator` exists
- `/toggle_navigation` exists
- `/robot_09/scan` has `Publisher count > 0`
- `/robot_09/cmd_vel_unstamped` has publisher `reactive_navigator`
- Motion command shows:
  - forward state: `linear.x ~ 0.05`, `angular.z ~ 0.0`
  - obstacle turn: `linear.x = 0.0`, `angular.z != 0.0`

## Launch arguments

```bash
ros2 launch lab4_obstacle_avoidance turtlebot_bringup.launch.py --show-args
```

- `safety_distance` (default: `0.5`)
- `scan_topic` (default: `/robot_09/scan`)
- `cmd_vel_topic` (default: `/robot_09/cmd_vel_unstamped`)
- `behavior_choice` (default: `C`, options: `A|B|C`)
- `choice_c_front_threshold` (default: `0.3`)
- `forward_speed` (default: `0.05`)
- `rotation_speed` (default: `0.8`)

## Example launches

```bash
ros2 launch lab4_obstacle_avoidance turtlebot_bringup.launch.py behavior_choice:=A safety_distance:=0.5
ros2 launch lab4_obstacle_avoidance turtlebot_bringup.launch.py behavior_choice:=B safety_distance:=0.5
ros2 launch lab4_obstacle_avoidance turtlebot_bringup.launch.py behavior_choice:=C choice_c_front_threshold:=0.3 rotation_speed:=1.0
```

## What NOT to do

- Do **not** mix discovery modes between robot and VM.
  - If VM uses `ROS_DISCOVERY_SERVER`, robot must use compatible discovery-server mode too.
- Do **not** set `ROS_DISCOVERY_SERVER=127.0.0.1:11811` on VM.
  - `127.0.0.1` points to VM itself, not the robot.
- Do **not** keep a trailing semicolon in discovery server value.
  - Bad: `ROS_DISCOVERY_SERVER=127.0.0.1:11811;`
- Do **not** leave stale daemon state after env changes.
  - Always run `ros2 daemon stop && ros2 daemon start`.
- Do **not** launch lab4 before robot base stack publishes `/robot_09/scan`.
- Do **not** use `/cmd_vel` in this setup; use `/robot_09/cmd_vel_unstamped`.

## Quick recovery when topics disappear

1. On robot: re-apply robot env and confirm `/robot_09/scan` publisher count > 0.
2. On VM: re-apply VM env and restart daemon.
3. Verify discovery values:

```bash
env | grep -E '^ROS_|^RMW_' | sort
```

4. Confirm VM can see robot topics:

```bash
ros2 topic list | head
ros2 topic info /robot_09/scan
```
