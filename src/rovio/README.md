ROVIO (stub) — ROS 2 package

This package contains a minimal stub wrapper for ROVIO as a ROS 2 node. It subscribes to a camera image topic and an IMU topic and publishes a placeholder `nav_msgs/Odometry` and TF.

How to build

From workspace root:

```bash
colcon build --packages-select rovio
```

How to run

```bash
# source your workspace then:
ros2 launch rovio rovio_launch.py
```

Next steps

- Replace the stub logic in `src/rovio_node.cpp` with actual calls to the ROVIO estimator.
- Add proper camera intrinsics handling, message synchronization, and estimator parameter mapping.
- Add integration tests using rosbag playback.
