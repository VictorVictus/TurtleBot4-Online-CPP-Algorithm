# TB4 Autonomous Coverage System

MATLAB-based autonomous coverage system for the TurtleBot4 (TB4) platform. The robot explores an area, detects ArUco markers, and then performs a boustrophedon coverage pattern using a camera cone model. A live GUI shows the camera feed, coverage map, and telemetry.

## Features

- ArUco marker detection and mapping (6x6 250 dictionary)
- Automatic boustrophedon path generation after all markers are found
- Real-time coverage area tracking and percentage display
- Obstacle avoidance using stereo depth data
- Bumper emergency recovery (reverse → 360° spin → 180° flip → forward nudge)
- Camera cone visualization on the coverage map
- ROS2 communication with compressed image topic and odometry relay
- GUI with dashboard showing position, heading, FPS, depth, and mission progress

## Requirements

- MATLAB R2022b or later with:
  - ROS Toolbox
  - Computer Vision Toolbox
  - Image Processing Toolbox
- Running ROS2 network (domain ID 0)
- TurtleBot4 with:
  - OAK-D camera publishing `/oakd/rgb/image_raw/compressed`
  - Stereo depth on `/oakd/stereo/image_raw`
  - Relay nodes (provided in this repository) that remap TB4 topics to:
    - `/pc_odom` (`nav_msgs/Odometry`)
    - `/bumper_hit` (`std_msgs/Bool`)
  - Velocity command subscriber `/pc_cmd_vel` (`geometry_msgs/Twist`)
- Calibration file `cameraparams3.mat` (intrinsic parameters for 1280×720 resolution)

## Setup

### 1. TB4 Relay Initialisation

The repository includes four Python relay scripts that must run on the TurtleBot4 to bridge topics.  
Copy them to the TB4 and run each one with `python3`.

#### Step-by-step from your local machine

Navigate to the folder containing the relay files (e.g., `relays/` in the repo), then transfer them to the TB4 and start the relays:

```bash
# On your local machine, inside the repository folder that contains:
#   cmd_vel_relay.py
#   hazard_bridge.py
#   hazard_relay_node.py
#   odom_relay.py

# 1. Copy all relay scripts to the TB4 (adjust IP and path if needed)
scp *.py ubuntu@<tb4-ip>:/home/ubuntu/relays/

# 2. SSH into the TB4 and start the relays
ssh ubuntu@<tb4-ip>

# On the TB4:
cd /home/ubuntu/relays

# It’s easiest to run each node in a separate terminal or in a tmux session.
# Example using tmux (all four nodes in a single session):
tmux new-session -d -s relays
tmux send-keys -t relays:0 "python3 odom_relay.py" C-m
tmux split-window -t relays:0 -v
tmux send-keys -t relays:0.1 "python3 hazard_bridge.py" C-m
tmux split-window -t relays:0.1 -h
tmux send-keys -t relays:0.2 "python3 hazard_relay_node.py" C-m
tmux split-window -t relays:0.0 -h
tmux send-keys -t relays:0.3 "python3 cmd_vel_relay.py" C-m

# Detach from tmux with Ctrl+B, D.
# To reattach later: tmux attach -t relays
