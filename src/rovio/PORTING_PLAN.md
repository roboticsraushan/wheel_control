Porting ethz-asl/rovio to ROS2 (overview)

Goal
- Integrate the original ethz-asl/rovio into this ROS2 workspace and provide a usable ament/ROS2 node that consumes camera + IMU and publishes `nav_msgs/Odometry` + TF.

High-level steps
1. Vendor the upstream repo into `src/rovio/third_party/rovio` (done).
2. Inventory upstream dependencies (kindr, lightweight_filtering, Eigen, OpenCV, Ceres?, etc.).
3. Add or port required dependencies into workspace (prefer ament packages or vendor minimal pieces):
   - `kindr` (ethz-asl/kindr) — port or add ament wrapper if needed.
   - `lightweight_filtering` (used as submodule) — vendor or port.
4. Replace catkin-specific macros in upstream CMakeLists (catkin_package, etc.) with ament-friendly equivalents OR create an ament wrapper CMakeLists that builds the rovio sources as a library.
5. Port ROS API usage (ros::NodeHandle, rosparam, ros::Subscriber) to rclcpp equivalents in the wrapper node if necessary; alternatively keep upstream code unchanged and write a small ROS2-compatible shim for runtime parameter loading and topic adapters.
6. Build iteratively and fix compile/link issues.
7. Add a ROS2 node wrapper `rovio_node` that instantiates the ROVIO estimator and exposes necessary parameters (camera/IMU topics, extrinsics, rovio.cfg mapping).
8. Create launch, params, and testing rosbag + CI steps.

Short-term plan (next actions I will perform now)
- Step A: Add `kindr` and `lightweight_filtering` quick inventory (check upstream submodules and includes) and note missing packages.
- Step B: Create a minimal ament wrapper CMakeLists in `src/rovio/third_party/rovio/ament/` that builds rovio as a static library (experimental).
- Step C: Attempt a first build and capture errors.

Notes & risks
- This is moderately high-effort: expect iterative fixes and dependency porting.
- I will aim to minimize scope initially (build rovio core without OpenGL scene, disable extras) and then progressively enable features.
