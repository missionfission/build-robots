# Site Inspector Robot

An autonomous mobile robot designed for construction site monitoring, inspection, and documentation.

## Features

- Autonomous site navigation
- 3D scanning and mapping
- Progress monitoring
- Safety hazard detection
- Quality inspection
- Documentation generation
- Real-time reporting

## Robot Components

### Mechanical Structure
- All-terrain tracked base
- Telescopic sensor mast
- Weatherproof housing
- Stabilization system
- Charging station dock
- Protective caging

### Sensors
- 3D LiDAR scanner
- High-resolution cameras
- Thermal imaging camera
- Environmental sensors
  - Dust particles
  - Noise levels
  - Temperature
  - Humidity
- GPS/RTK positioning
- IMU

### Control Systems
- Autonomous navigation
- Inspection planning
- Data collection
- Hazard identification
- Progress tracking
- Report generation

## Usage

1. Launch the system:
```bash
ros2 launch construction_robots site_inspector.launch.py
```

2. Start inspection mission:
```bash
ros2 run construction_robots start_inspection
```

3. View inspection data:
```bash
ros2 launch construction_robots inspection_visualization.launch.py
```

## Configuration

Configure through:
- `config/inspection_params.yaml`: Inspection parameters
- `config/robot_config.yaml`: Robot configuration
- `config/sensor_config.yaml`: Sensor settings
- `config/reporting_config.yaml`: Report generation settings

## Operation Modes

1. **Autonomous Inspection**
   - Scheduled site surveys
   - Automated data collection
   - Real-time reporting

2. **Manual Control**
   - Direct robot control
   - Targeted inspection
   - Data collection

3. **Emergency Mode**
   - Hazard detection
   - Emergency notification
   - Site evacuation support

## Safety Features

- Obstacle avoidance
- Hazard detection
- Emergency stop
- Weather protection
- Remote monitoring
- Geofencing

## Dependencies

- ROS2 Foxy/Galactic
- Nav2
- OpenCV
- PCL
- TensorFlow
- Reporting tools
- Gazebo 