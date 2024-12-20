# Monitoring Drone

An autonomous aerial drone for crop monitoring and data collection in urban farming environments.

## Features

- Autonomous flight control
- Multi-spectral imaging
- Plant health monitoring
- Environmental sensing
- Automated data collection
- Real-time mapping
- Precision landing

## Robot Components

### Mechanical Structure
- Quadcopter frame
- Protected propellers
- Weatherproof housing
- Modular sensor payload
- Automated charging dock
- LED indicators

### Sensors
- Multi-spectral camera
- Thermal imaging camera
- GPS/RTK positioning
- IMU
- Ultrasonic sensors
- Environmental sensors
  - Temperature
  - Humidity
  - CO2
  - Light intensity

### Control Systems
- Autonomous navigation
- Waypoint planning
- Obstacle avoidance
- Data collection scheduling
- Emergency protocols
- Precision landing system

## Usage

1. Launch the simulation:
```bash
ros2 launch urban_farming_robots monitoring_drone.launch.py
```

2. Start monitoring mission:
```bash
ros2 run urban_farming_robots start_monitoring
```

3. View real-time data:
```bash
ros2 launch urban_farming_robots monitoring_visualization.launch.py
```

## Configuration

Configure through:
- `config/flight_params.yaml`: Flight parameters
- `config/sensor_config.yaml`: Sensor settings
- `config/mission_config.yaml`: Mission planning
- `config/data_collection.yaml`: Data collection settings

## Operation Modes

1. **Autonomous Mission**
   - Scheduled monitoring
   - Automated data collection
   - Return to dock

2. **Manual Control**
   - Direct flight control
   - Manual data capture
   - Testing and calibration

3. **Emergency Mode**
   - Return to home
   - Emergency landing
   - Failsafe protocols

## Safety Features

- Obstacle detection and avoidance
- Low battery return
- GPS loss protocols
- Wind speed monitoring
- Emergency landing
- Geofencing

## Dependencies

- ROS2 Foxy/Galactic
- PX4 Autopilot
- OpenCV
- Geographic Lib
- PCL
- Gazebo 