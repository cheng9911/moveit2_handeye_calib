#!/bin/bash

# Clone Franka dependencies into the workspace
vcs import /ros2_ws/src < /ros2_ws/src/franka.repos --recursive --skip-existing

exec "$@"