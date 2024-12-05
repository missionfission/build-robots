# Simple Robot Manipulator Example

This example demonstrates a basic 4-DOF robot manipulator with the following features:
- Rotating base (continuous rotation)
- 3-link arm (shoulder, elbow, wrist)
- Simple gripper mount
- Full visualization in RViz and Gazebo

## Robot Structure

```
base_link
    └── arm_base (continuous rotation)
        └── upper_arm (shoulder joint: ±90°)
            └── forearm (elbow joint: ±90°)
                └── gripper_base (wrist joint: ±90°)
```

## Features Demonstrated
- URDF robot description with proper inertial properties
- Joint limits and continuous joints
- Multiple geometric primitives (cylinders and boxes)
- Color materials
- Integration with RViz and Gazebo
- Joint trajectory control visualization
- Modern Python-based launch system

## Launch Files
The example includes both XML and Python-based launch files:

### XML Launch (Legacy)
```bash
roslaunch robot_template/examples/simple_robot/simple_robot.launch
```

### Python Launch (Modern)
```bash
ros2 launch robot_template/examples/simple_robot/launch_sim.py
```

The Python launch system provides more flexibility with:
- Configurable simulation parameters
- Headless mode support
- Namespace management
- Conditional node launching
- Better error handling

### Launch Arguments
The Python launch file supports various arguments:
- `namespace`: Top-level namespace (default: '')
- `use_namespace`: Apply namespace to navigation stack (default: false)
- `headless`: Run without Gazebo client GUI (default: false)
- `use_rviz`: Start RViz visualization (default: true)
- `use_sim_time`: Use simulation clock (default: true)
- `world`: Custom world file path

Example with arguments:
```bash
ros2 launch robot_template/examples/simple_robot/launch_sim.py headless:=true use_rviz:=false
```

## Running the Example

1. Make sure you have completed the main setup:
```bash
./scripts/setup.sh
```

2. Choose your preferred launch method (XML or Python) and run the simulation.

3. Use the joint_state_publisher_gui to move the robot:
- The sliders control each joint
- Watch the robot move in both RViz and Gazebo
- Use RQT tools to monitor the robot's state

## Extending the Example

You can use this example as a starting point for your own robot by:
1. Modifying the URDF file to change the robot's structure
2. Adding more links and joints
3. Customizing the visual and collision properties
4. Adding sensors and actuators
5. Implementing custom controllers
6. Creating custom launch configurations