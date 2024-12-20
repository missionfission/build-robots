# Lifting Helper Robot

A specialized robot designed to assist with safe patient transfer and mobility support in elderly care environments.

## Features

- Safe patient lifting
- Posture monitoring
- Fall prevention
- Voice interaction
- Emergency response
- Mobility assistance
- Vital sign monitoring

## Robot Components

### Mechanical Structure
- Powered lifting arms
- Adjustable support frame
- Soft-touch grippers
- Ergonomic handles
- Stabilizing base
- Padding and cushioning
- Emergency release system

### Sensors
- Force-torque sensors
- Pressure sensors
- Motion capture cameras
- Vital sign monitors
- Proximity sensors
- Balance sensors
- Position encoders

### Control Systems
- Lift path planning
- Balance control
- Force adaptation
- Safety monitoring
- Voice commands
- Emergency detection
- User recognition

## Usage

1. Launch the system:
```bash
ros2 launch elderly_care_robots lifting_helper.launch.py
```

2. Start assistance mode:
```bash
ros2 run elderly_care_robots start_assistance
```

3. Monitor operations:
```bash
ros2 launch elderly_care_robots assistance_visualization.launch.py
```

## Configuration

Configure through:
- `config/lifting_params.yaml`: Lifting parameters
- `config/safety_config.yaml`: Safety settings
- `config/patient_profiles.yaml`: Patient information
- `config/movement_patterns.yaml`: Movement patterns

## Operation Modes

1. **Assisted Lifting**
   - Guided patient transfer
   - Force amplification
   - Balance support

2. **Training Mode**
   - Movement practice
   - Strength building
   - Balance training

3. **Emergency Mode**
   - Fall prevention
   - Emergency support
   - Quick response

## Safety Features

- Load monitoring
- Balance detection
- Emergency stop
- Soft collision detection
- Vital sign monitoring
- Voice feedback
- Fail-safe mechanisms

## Special Capabilities

- Multi-point lifting
- Adaptive force control
- Gait analysis
- Progress tracking
- Remote monitoring
- Exercise guidance
- Recovery assistance

## Dependencies

- ROS2 Foxy/Galactic
- Moveit2
- OpenCV
- PCL
- Healthcare APIs
- Force control library
- Safety monitoring system 