# Go2 Front Camera Feed

## Package Structure

```
my_ros2_ws/src/go2_gps_nav/
├── go2_gps_nav/
│   ├── __init__.py
│   └── gps_navigation_node.py   # Main navigation code
├── config/
│   └── navigation_params.yaml   # Configuration parameters
├── launch/
│   └── gps_navigation.launch.py # Launch file
├── resource/
│   └── go2_gps_nav              # Marker file for package discovery
├── package.xml
├── setup.py
└── setup.cfg
```

## Dependencies

- ROS2 Foxy
- Python 3

## Usage

1. Launch the navigation node:
   ```bash
   ros2 launch go2_gps_nav gps_navigation.launch.py
   ```

2. Send a GPS goal:
   ```bash
   ros2 topic pub -1 /goal sensor_msgs/msg/NavSatFix "{latitude: 38.8277645, longitude: -77.3051732999}"
   ```


### Published Topics
- `/cmd_vel` ([geometry_msgs/Twist](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Twist.html)) - Robot velocity commands
