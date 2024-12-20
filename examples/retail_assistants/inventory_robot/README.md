# Inventory Robot

An autonomous mobile robot designed for automated inventory tracking and management in retail environments.

## Features

- Real-time inventory scanning
- Barcode/RFID reading
- Misplaced item detection
- Price verification
- Stock level monitoring
- Planogram compliance
- Data analytics

## Robot Components

### Mechanical Structure
- Compact mobile base
- Telescopic scanner array
- Multi-angle cameras
- LED lighting system
- Charging dock interface
- Low-profile design

### Sensors
- RFID scanner array
- Barcode readers
- High-resolution cameras
- 2D LiDAR
- Proximity sensors
- Touch sensors

### Control Systems
- Autonomous navigation
- Inventory mapping
- Pattern recognition
- Database integration
- Analytics processing
- Real-time reporting

## Usage

1. Launch the system:
```bash
ros2 launch retail_robots inventory_robot.launch.py
```

2. Start inventory scan:
```bash
ros2 run retail_robots start_inventory
```

3. View inventory data:
```bash
ros2 launch retail_robots inventory_visualization.launch.py
```

## Configuration

Configure through:
- `config/scanning_params.yaml`: Scanning parameters
- `config/robot_config.yaml`: Robot configuration
- `config/inventory_db.yaml`: Database settings
- `config/store_layout.yaml`: Store mapping

## Operation Modes

1. **Full Store Scan**
   - Complete inventory check
   - Automated reporting
   - Analytics generation

2. **Section Scan**
   - Targeted area scanning
   - Quick stock checks
   - Price verification

3. **Search Mode**
   - Missing item location
   - Misplaced item detection
   - Specific product search

## Features

- Real-time inventory updates
- Missing item alerts
- Price discrepancy detection
- Stock level warnings
- Planogram compliance checking
- Historical data tracking

## Safety Features

- Customer detection
- Obstacle avoidance
- Low-speed operation
- Emergency stops
- Quiet operation
- Work zone monitoring

## Dependencies

- ROS2 Foxy/Galactic
- Nav2
- OpenCV
- TensorFlow
- Database systems
- Inventory management software
- Gazebo 