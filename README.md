# Go2 Front Camera Feed

## Package Structure

```
my_ros2_ws/src/go2_camra/
├── go2_camera/
│   ├── __init__.py
│   └── camera_node.py   # Main navigation code
├── launch/
│   └── camera_feed.launch.py # Launch file
├── resource/
│   └── go2_camera              # Marker file for package discovery
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
   ros2 launch go2_camera camera_feed.launch.py
   ```

### Published Topics
- `/go2_video_frames topic` ([sensor_msgs/Image](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)) - Image topic for Rviz

### Generate Nodes
- `/camera_node`
