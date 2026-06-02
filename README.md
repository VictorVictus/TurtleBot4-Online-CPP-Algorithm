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
Before running the MATLAB script, start the relay nodes on the TurtleBot4. These relays convert the TB4's native topics into the standardised names expected by the code. The relay scripts are included in this repository (see the `relays` folder). Run them on the TB4 using the provided launch files or scripts, for example:

```bash
# On the TurtleBot4
cd ~/tb4-relays
ros2 launch tb4_relays all_relays.launch.py
