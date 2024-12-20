# Waste Sorting Robot

An automated system for waste classification and sorting using computer vision and robotic manipulation.

## Features

- Multi-stream waste classification
- High-speed conveyor system
- AI-powered material recognition
- Robotic picking and sorting
- Contamination detection
- Throughput monitoring
- Safety systems

## Robot Components

### Mechanical Structure
- Dual-arm delta robots
- Multi-lane conveyor system
- Material separation mechanisms
- Collection bins
- Safety enclosure
- Maintenance access points

### Perception System
- High-speed RGB cameras
- Hyperspectral imaging
- 3D depth sensors
- Metal detectors
- Weight sensors
- NIR (Near-Infrared) sensors

### Sorting Capabilities
- Plastics (PET, HDPE, PVC, etc.)
- Metals (Ferrous and Non-ferrous)
- Paper and Cardboard
- Glass
- Organic Materials
- E-waste
- Hazardous Materials

## Usage

1. Start the sorting system:
```bash
ros2 launch waste_robots sorting_system.launch.py
```

2. Initialize vision system:
```bash
ros2 launch waste_robots vision_processing.launch.py
```

3. Monitor sorting metrics:
```bash
ros2 launch waste_robots sorting_metrics.launch.py
```

## Operation Modes

1. **Full Automatic Mode**
   - Continuous sorting operation
   - Automatic bin management
   - Real-time throughput optimization

2. **Maintenance Mode**
   - System cleaning and calibration
   - Component testing
   - Sensor calibration

3. **Training Mode**
   - New material training
   - Vision system updates
   - Sorting pattern optimization

## Configuration

System parameters in:
- `config/vision_params.yaml`: Vision system settings
- `config/sorting_config.yaml`: Sorting parameters
- `config/conveyor_config.yaml`: Conveyor settings
- `config/robot_config.yaml`: Robot arm configuration

## Performance Metrics

- Sorting accuracy: >95%
- Throughput: 1000 items/hour
- Material streams: 8+
- Contamination detection: 99%
- Recovery rate: >90%

## Safety Features

- Emergency stops
- Light curtains
- Safety interlocks
- Automatic fault detection
- Contamination containment
- Fire detection system

## Dependencies

- ROS2 Foxy/Galactic
- OpenCV
- TensorFlow
- PyTorch
- PCL
- Moveit2
- Industrial control packages 