# Shelf Stocking Robot

An autonomous robot designed for automated inventory management and shelf restocking in retail environments.

## Features

- Automated product placement
- Real-time inventory tracking
- Planogram compliance checking
- Product recognition
- Gap detection
- Price tag verification
- Customer interaction safety

## Robot Components

### Mechanical Structure
- Omnidirectional mobile base
- Telescopic lift system
- Multi-joint manipulation arm
- Product gripper array
- Product storage compartments
- Safety bumpers

### Sensors
- 3D vision system
- Barcode/RFID scanners
- Force-torque sensors
- Proximity sensors
- Touch sensors
- Product weight sensors

### Control Systems
- Shelf space optimization
- Product placement planning
- Navigation in retail spaces
- Real-time inventory updates
- Customer detection
- Safety monitoring

## Usage

1. Launch the system:
```bash
ros2 launch retail_robots shelf_stocker.launch.py
```

2. Start stocking operation:
```bash
ros2 run retail_robots start_stocking
```

3. Monitor operations:
```bash
ros2 launch retail_robots stocking_visualization.launch.py
```

## Configuration

Configure through:
- `config/stocking_params.yaml`: Stocking parameters
- `config/robot_config.yaml`: Robot configuration
- `config/product_db.yaml`: Product database
- `config/planogram.yaml`: Shelf layout settings

## Operation Modes

1. **Autonomous Stocking**
   - Continuous shelf monitoring
   - Automated restocking
   - Planogram compliance

2. **Manual Control**
   - Direct robot control
   - Product placement testing
   - System calibration

3. **Inventory Mode**
   - Stock level checking
   - Product verification
   - Database updates

## Safety Features

- Customer detection
- Collision avoidance
- Load monitoring
- Emergency stops
- Speed limiting
- Work zone monitoring

## Dependencies

- ROS2 Foxy/Galactic
- Moveit2
- OpenCV
- TensorFlow
- PCL
- Gazebo
- Industrial control packages 