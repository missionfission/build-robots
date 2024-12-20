# Mobile Assistant Robot

An intelligent mobile robot designed to assist elderly individuals with daily activities and provide continuous monitoring and support.

## Features

- Omnidirectional mobile base
- Interactive touch screen interface
- Voice interaction system
- Object manipulation capabilities
- Fall detection and emergency response
- Vital signs monitoring
- Navigation and mapping

## Robot Components

### Mechanical Structure
- Holonomic drive base
- Adjustable height display
- Dual-arm manipulation system
- Storage compartments
- Charging dock interface
- Safety bumpers and covers

### Sensors
- 2D LiDAR for navigation
- RGB-D cameras for perception
- Force-torque sensors in arms
- Vital sign sensors
  - Temperature sensor
  - Heart rate monitor
  - Blood pressure sensor
- Emergency button
- Touch sensors

### Interactive Systems
- Touch screen display
- Voice recognition system
- Text-to-speech output
- Gesture recognition
- Emergency alert system
- Remote monitoring interface

## Usage

1. Launch the robot system:
```bash
ros2 launch elderly_care_robots mobile_assistant.launch.py
```

2. Start the navigation system:
```bash
ros2 launch elderly_care_robots navigation.launch.py map:=home_layout.yaml
```

3. Launch the user interface:
```bash
ros2 launch elderly_care_robots assistant_ui.launch.py
```

## Operation Modes

1. **Follow Mode**
   - Follows the user maintaining safe distance
   - Carries items and provides support
   - Continuous vital monitoring

2. **Autonomous Mode**
   - Independent navigation
   - Scheduled task execution
   - Environmental monitoring

3. **Emergency Mode**
   - Fall detection response
   - Emergency contact notification
   - Medical data transmission
   - Two-way communication

## Configuration

Customize the robot through:
- `config/navigation_params.yaml`: Navigation settings
- `config/interaction_config.yaml`: UI and voice settings
- `config/monitoring_config.yaml`: Health monitoring parameters
- `config/emergency_config.yaml`: Emergency response settings

## Safety Features

- Emergency stop system
- Collision avoidance
- Battery level monitoring
- Vital sign alerts
- Fall detection
- Remote shutdown capability

## Dependencies

- ROS2 Foxy/Galactic
- Nav2
- OpenCV
- TensorFlow
- PyTorch
- Qt5
- Web interface packages 