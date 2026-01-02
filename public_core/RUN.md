````markdown
# RUN.md

---

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- RViz2

---

## Environment Setup

```bash
source /opt/ros/jazzy/setup.bash
````

---

## Start Swarm Core

### Terminal 1

```bash
python3 public_core/swarm_core.py
```

---

## ROS Interfaces

### Published Topics

* `/swarm/markers` — visualization_msgs/MarkerArray
* `/swarm/metrics` — std_msgs/String

### Subscribed Topics

* `/swarm/cmd` — std_msgs/String

---

## Control Commands

### Terminal 2

```bash
source /opt/ros/jazzy/setup.bash
```

### Reset Simulation

```bash
ros2 topic pub --once /swarm/cmd std_msgs/msg/String "{data: reset}"
```

### Start Coverage Mission

```bash
ros2 topic pub --once /swarm/cmd std_msgs/msg/String "{data: coverage}"
```

### Monitor Metrics

```bash
ros2 topic echo /swarm/metrics std_msgs/msg/String
```

---

## RViz Visualization

### Launch RViz2

```bash
rviz2
```

### Add Display

* Display Type: MarkerArray
* Topic: `/swarm/markers`

---

## Visualization Elements

* Blue spheres — swarm agents
* Gold sphere — coordinator agent
* White lines — communication links
* Yellow sphere — shared swarm_goal
* Red spheres — local goals (SWEEP phase only)
* Text marker — phase, coverage %, connectivity

---

## Regroup Logic

### Trigger Condition

* connectivity < total number of agents

### Actions

* force TRAVEL phase
* suppress local goals
* pull agents toward coordinator

---

## Mission Completion

### Conditions

* coverage ≥ target threshold
* condition sustained for defined duration

### Result

* mode switches to RETURN
* swarm moves toward home position

---

## Scope

* simulation-only
* no physical dynamics
* no collision avoidance
* no hardware control

---

## Licensing

* Default: All Rights Reserved
* Alternative: Apache License 2.0 (by permission)
* See: LICENSING.md, CONTACT.md

