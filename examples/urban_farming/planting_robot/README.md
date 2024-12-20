# Automated Planting Robot

A specialized robot designed for automated seedling planting and soil preparation in urban farming environments.

## Features

- 6-DOF robotic arm for precise planting
- Soil preparation end-effector
- Seed/seedling dispensing system
- Computer vision for plant placement
- Soil sensor integration
- Mobile base for greenhouse navigation

## Robot Components

### Mechanical Structure
- Mobile base (4-wheel differential drive)
- Telescopic lift for height adjustment
- 6-DOF robotic arm
- Custom end-effector with:
  - Soil preparation tools
  - Seed dispensing mechanism
  - Watering system

### Sensors
- RGB-D camera for plant placement
- Soil moisture and pH sensors
- Proximity sensors for obstacle avoidance
- Wheel encoders for odometry
- IMU for balance and orientation

### Control Systems
- Path planning for greenhouse navigation
- Precision arm control for planting
- Soil preparation sequence
- Plant spacing optimization
- Safety monitoring system

## Usage

1. Launch the simulation:
```bash
ros2 launch urban_farming_robots planting_robot.launch.py
```

2. Start the planting sequence:
```bash
ros2 run urban_farming_robots start_planting
```

3. Monitor through RViz:
```bash
ros2 launch urban_farming_robots planting_visualization.launch.py
```

## Configuration

The robot can be configured through:
- `config/planting_params.yaml`: Planting parameters
- `config/robot_config.yaml`: Robot configuration
- `config/sensors_config.yaml`: Sensor settings

## Operation Modes

1. **Autonomous Mode**
   - Automated row-by-row planting
   - Obstacle avoidance
   - Self-charging

2. **Manual Mode**
   - Teleoperation control
   - Individual plant placement
   - System testing and calibration

## Dependencies

- ROS2 Foxy/Galactic
- Moveit2
- OpenCV
- PCL (Point Cloud Library)
- Gazebo 