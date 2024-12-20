# Cleaning Robot

An autonomous robot designed for efficient floor cleaning and sanitization in various environments.

## Features

- Multi-surface cleaning
- Autonomous navigation
- Dirt detection
- Water management
- Sanitization
- Obstacle avoidance
- Schedule management

## Robot Components

### Mechanical Structure
- Compact mobile base
- Dual brush system
- Water tanks
  - Clean water
  - Dirty water
- UV sanitization module
- Squeegee system
- Debris container
- Auto-dock system

### Sensors
- 3D LiDAR for navigation
- Dirt detection cameras
- Moisture sensors
- Fill level sensors
- Surface type detection
- Obstacle detection
- Cliff sensors

### Control Systems
- Path planning
- Surface adaptation
- Cleaning pattern optimization
- Water flow control
- Battery management
- Schedule execution
- Performance monitoring

## Usage

1. Launch the system:
```bash
ros2 launch waste_robots cleaning_robot.launch.py
```

2. Start cleaning operation:
```bash
ros2 run waste_robots start_cleaning
```

3. Monitor operations:
```bash
ros2 launch waste_robots cleaning_visualization.launch.py
```

## Configuration

Configure through:
- `config/cleaning_params.yaml`: Cleaning parameters
- `config/robot_config.yaml`: Robot configuration
- `config/schedule_config.yaml`: Schedule settings
- `config/area_maps.yaml`: Area definitions

## Operation Modes

1. **Autonomous Cleaning**
   - Scheduled operation
   - Area coverage
   - Adaptive cleaning

2. **Spot Cleaning**
   - Targeted cleaning
   - Deep cleaning
   - Stain removal

3. **Maintenance Mode**
   - Tank filling/emptying
   - Filter cleaning
   - System checks

## Special Capabilities

- Multi-floor operation
- Different surface types
- Stain detection
- UV disinfection
- Remote monitoring
- Performance analytics
- Resource optimization

## Safety Features

- Obstacle avoidance
- Wet floor detection
- Cliff prevention
- Battery monitoring
- Leak detection
- Emergency stop
- Anti-tamper system

## Dependencies

- ROS2 Foxy/Galactic
- Nav2
- OpenCV
- PCL
- Fleet management
- Scheduling system
- Monitoring interface 