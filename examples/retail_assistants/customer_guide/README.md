# Customer Guide Robot

An interactive mobile robot designed to assist customers with navigation, product location, and information in retail environments.

## Features

- Interactive guidance
- Store navigation
- Product locating
- Multi-language support
- Customer service
- Queue management
- Store information

## Robot Components

### Mechanical Structure
- Compact mobile base
- Interactive display
- Gesture sensors
- LED indicators
- Speaker system
- Microphone array
- Aesthetic design

### Sensors
- 2D LiDAR for navigation
- Depth cameras
- Touch screen
- Gesture recognition
- Voice recognition
- People detection
- Proximity sensors

### Control Systems
- Navigation planning
- Speech processing
- Customer tracking
- Store mapping
- Queue detection
- Interaction management
- Behavior planning

## Usage

1. Launch the system:
```bash
ros2 launch retail_robots customer_guide.launch.py
```

2. Start guide service:
```bash
ros2 run retail_robots start_guide
```

3. Monitor operations:
```bash
ros2 launch retail_robots guide_visualization.launch.py
```

## Configuration

Configure through:
- `config/interaction_params.yaml`: Interaction parameters
- `config/navigation_config.yaml`: Navigation settings
- `config/store_map.yaml`: Store layout
- `config/product_db.yaml`: Product database

## Operation Modes

1. **Active Guide**
   - Customer following
   - Product location
   - Store tours

2. **Information Point**
   - Stationary assistance
   - Queue management
   - Information desk

3. **Roaming Mode**
   - Proactive assistance
   - Customer detection
   - Service offering

## Special Capabilities

- Natural language processing
- Multi-language support
- Gesture recognition
- Queue management
- Product recommendations
- Store analytics
- Customer behavior tracking

## Safety Features

- Obstacle avoidance
- Crowd detection
- Speed control
- Emergency stop
- Personal space awareness
- Child detection
- Collision prevention

## Dependencies

- ROS2 Foxy/Galactic
- Nav2
- OpenCV
- TensorFlow
- Speech recognition
- NLP libraries
- Web services
- Store management system 