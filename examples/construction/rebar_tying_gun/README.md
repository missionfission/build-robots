# Rebar Tying Assistive Gun Robot

An assistive robot designed for automated rebar tying in construction projects, specifically targeting real estate construction sites.

## Features

- Precision rebar wire tying
- Adjustable tie tension control
- Pattern recognition for rebar intersections
- Ergonomic handheld design
- Battery-powered operation
- Quick tie completion (2-3 seconds per tie)
- Wire feeding system
- Safety mechanisms

## Robot Components

### Mechanical Structure
- Lightweight handheld frame (2.5kg)
- Rotating tying mechanism
- Wire feeding system
- Adjustable grip handle
- Wire storage compartment
- Tension adjustment dial
- Emergency stop button

### Sensors
- Computer vision camera
- Proximity sensors
- Force feedback sensors
- Wire tension sensors
- Battery level indicator
- Tie completion detector

### Control Systems
- Rebar intersection detection
- Automated wire feeding
- Tie tension control
- Pattern learning system
- User interface
- Safety monitoring

## Usage

1. Launch the system:
```bash
ros2 launch construction_robots rebar_tying_gun.launch.py
```

2. Start tying operation:
```bash
ros2 run construction_robots start_rebar_tying
```

3. Monitor operations:
```bash
ros2 launch construction_robots tying_visualization.launch.py
```

## Configuration

Configure through:
- `config/tying_params.yaml`: Tying parameters
- `config/robot_config.yaml`: Robot configuration
- `config/vision_config.yaml`: Computer vision settings
- `config/safety_config.yaml`: Safety parameters

## Operation Modes

1. **Assisted Mode**
   - User positions gun at intersection
   - Automatic tie execution
   - Visual feedback for positioning

2. **Semi-Automatic Mode**
   - Continuous operation along rebar line
   - Automatic intersection detection
   - Hands-free operation

3. **Manual Override**
   - Direct user control
   - Custom tie patterns
   - Maintenance mode

## Safety Features

- Emergency stop mechanism
- Wire jam detection
- Battery monitoring
- Overheating protection
- User safety alerts
- Proper grip detection

## Technical Specifications

- **Weight**: 2.5kg (including battery)
- **Battery Life**: 8-10 hours continuous operation
- **Tie Speed**: 2-3 seconds per tie
- **Wire Compatibility**: 16-20 gauge steel wire
- **Operating Temperature**: -10°C to 50°C
- **IP Rating**: IP65 (dust and water resistant)

## Target Applications

- Residential construction
- Commercial buildings
- Infrastructure projects
- Real estate developments
- Foundation work
- Concrete reinforcement

## Dependencies

- ROS2 Foxy/Galactic
- OpenCV
- PCL
- Industrial control packages
- Computer vision libraries
- Battery management system