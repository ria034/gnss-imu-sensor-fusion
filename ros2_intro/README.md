# ROS 2 Introduction

First hands-on assignment: install ROS 2, build a workspace, and write a
minimal publisher/subscriber package.

## Contents

- `src/py_pubsub/` — ROS 2 (ament_python) package with a `publisher_member_function`
  node and a `subscriber_member_function` node.
- `media/turtlesim.png` — screenshot from driving the Turtlesim demo node.
- `report/ros2_intro_notes.docx` — short submission notes (environment setup + Git/GitHub readiness check).

## Running

```bash
cd ros2_intro/src
colcon build --packages-select py_pubsub
source install/setup.bash
ros2 run py_pubsub talker
ros2 run py_pubsub listener
```
