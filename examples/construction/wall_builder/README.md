# Wall Builder Robot

An autonomous robot designed for automated brick laying and wall construction in construction environments.

## Features

- Precision brick placement
- Automated mortar application
- Real-time construction verification
- Structural integrity monitoring
- Progress tracking
- Safety compliance

## Robot Components

### Mechanical Structure
- Heavy-duty tracked base
- Telescopic main arm
- Brick gripper system
- Mortar applicator
- Material storage system
- Stabilizing outriggers

### Sensors
- 3D LiDAR scanner
- Stereo cameras
- Force-torque sensors
- Level sensors
- Proximity sensors
- Structural alignment sensors

### Control Systems
- Wall pattern planning
- Brick placement optimization
- Mortar thickness control
- Real-time quality checks
- Safety zone monitoring
- Progress tracking

## Usage

1. Launch the simulation:
```bash
ros2 launch construction_robots wall_builder.launch.py
```

2. Start construction sequence:
```bash
ros2 run construction_robots start_building
```

3. Monitor construction:
```bash
ros2 launch construction_robots building_visualization.launch.py
```

## Configuration

Configure through:
- `config/construction_params.yaml`: Construction parameters
- `config/robot_config.yaml`: Robot configuration
- `config/quality_checks.yaml`: Quality control settings
- `config/safety_config.yaml`: Safety parameters

## Operation Modes

1. **Autonomous Construction**
   - Continuous wall building
   - Automated material management
   - Quality control checks

2. **Manual Control**
   - Direct robot control
   - Individual brick placement
   - System calibration

3. **Inspection Mode**
   - Wall alignment checking
   - Mortar thickness verification
   - Structural integrity scanning

## Safety Features

- Work zone monitoring
- Load monitoring
- Stability control
- Emergency stops
- Collision avoidance
- Weather monitoring

## Dependencies

- ROS2 Foxy/Galactic
- Moveit2
- OpenCV
- PCL
- Gazebo
- Industrial control packages 