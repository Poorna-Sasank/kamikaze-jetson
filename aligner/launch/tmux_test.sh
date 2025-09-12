#!/bin/bash

source /home/hild/kamikaze/install/setup.bash  # Update with your workspace path

# Paths
PX4_DIR="/home/hild/PX4-Autopilot"  # Update with your PX4 path
QGC_PATH="/home/hild/Downloads/QGroundControl-x86_64.AppImage"  # Update with your QGC path

# Start a new tmux session named 'ros2_px4_session'
tmux new-session -d -s ros2_px4_session

# Split window vertically for MicroXRCE-DDS Agent (left pane)
tmux split-window -h
tmux select-pane -t 0
tmux send-keys "MicroXRCEAgent udp4 -p 8888" C-m

# Split the right pane vertically for PX4 SITL
tmux select-pane -t 1
tmux split-window -h
tmux select-pane -t 1
tmux send-keys "cd $PX4_DIR && make px4_sitl gz_x500_depth" C-m

# Split the rightmost pane vertically for QGroundControl
tmux select-pane -t 2
tmux split-window -h
tmux select-pane -t 2
tmux send-keys "$QGC_PATH" C-m

# Split the left pane horizontally for two ROS 2 nodes
tmux select-pane -t 0
tmux split-window -v
tmux select-pane -t 0
tmux send-keys "ros2 run object_detector object_detector_node" C-m
tmux select-pane -t 1
tmux send-keys "ros2 run aligner aligner" C-m

# Attach to the tmux session
tmux attach-session -t ros2_px4_session