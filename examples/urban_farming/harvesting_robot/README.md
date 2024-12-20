# Harvesting Robot

An autonomous robot designed for selective harvesting of crops in urban farming environments.

## Features

- Vision-based ripeness detection
- Dual-arm manipulation system
- Soft grippers for delicate handling
- Mobile base with omnidirectional movement
- Real-time produce quality assessment
- Bin management system

## Robot Components

### Mechanical Structure
- Omnidirectional mobile base
- Dual 7-DOF robotic arms
- Soft pneumatic grippers
- Produce collection bins
- Active suspension system

### Sensors
- Multi-spectral cameras
- Force-torque sensors
- 3D depth cameras
- LiDAR for navigation
- Tactile sensors in grippers

### Control Systems
- Ripeness detection AI
- Grasp planning
- Path optimization
- Bin fill-level monitoring
- Harvest tracking system

## Usage

1. Launch the simulation:
```bash
ros2 launch urban_farming_robots harvesting_robot.launch.py
```

2. Start the harvesting sequence:
```bash
ros2 run urban_farming_robots start_harvesting
```

3. Monitor through RViz:
```bash
ros2 launch urban_farming_robots harvesting_visualization.launch.py
```

## Configuration

The robot can be configured through:
- `config/harvesting_params.yaml`: Harvesting parameters
- `config/robot_config.yaml`: Robot configuration
- `config/vision_config.yaml`: Vision system settings

## Operation Modes

1. **Autonomous Mode**
   - Continuous harvesting
   - Automatic bin management
   - Quality control sorting

2. **Manual Mode**
   - Teleoperation control
   - Individual plant inspection
   - System calibration

## Dependencies

- ROS2 Foxy/Galactic
- Moveit2
- OpenCV
- PyTorch (for vision AI)
- PCL (Point Cloud Library)
- Gazebo 