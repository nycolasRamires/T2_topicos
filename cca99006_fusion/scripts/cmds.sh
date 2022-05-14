#!/bin/bash

ros2 topic pub -1 /twist_mrac_linearizing_controller/command geometry_msgs/msg/Twist "{linear: {x: 0.25, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.15}}"