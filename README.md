# wheel_control

Arduino Mega 2560 + Cytron driver control with a simple serial protocol and ROS 2 (Humble) bridge for teleoperation via keyboard or Logitech F310 gamepad.

## Workspace structure

```
wheel_control/
├── src/
│   └── wheel_control/          # ROS 2 Python package
│       ├── package.xml
│       ├── setup.py
│       ├── setup.cfg
│       ├── resource/
│       ├── wheel_control/      # Python modules
│       │   ├── __init__.py
│       │   ├── serial_motor_bridge.py
│       │   └── teleop_keyboard.py
│       ├── launch/             # Launch files
│       │   └── bringup.launch.py
│       └── config/             # Config files
│           └── teleop_f310.yaml
├── arduino/
│   └── wheel_control_mega/     # Arduino sketch
│       └── wheel_control_mega.ino
└── README.md
```

## Hardware

- Board: Arduino Mega 2560
- Motor driver: Cytron dual-channel (DIR + PWM inputs)
- Motor channels and pins:
	- Motor 1: DIR -> D7, PWM -> D8
	- Motor 2: DIR -> D9, PWM -> D10

Make sure D8 and D10 are wired to the PWM inputs of your Cytron driver and D7/D9 to the corresponding DIR pins. Supply appropriate motor power to the driver. Common ground between Arduino, driver logic GND, and any external controller is required.

## Arduino firmware

Location: `arduino/wheel_control_mega/wheel_control_mega.ino`

Features:
- Serial protocol over USB at 115200 baud
	- `M <left> <right>`: set motor PWM for left and right in range [-255, 255]
	- `S`: stop both motors
	- `P`: ping (Arduino replies `PONG`)
- Watchdog stops motors if no valid command arrives within 500 ms.
- Direction inversion flags per motor inside the sketch for easy wiring correction.

Flash steps:
1. Open the `.ino` in Arduino IDE or Arduino CLI.
2. Board: Arduino Mega or Mega 2560, Port: the detected USB port.
3. Upload.

Quick test (USB serial):
```
# Send forward on both motors for a second, then stop
M 150 150
S
```

## ROS 2 Humble bridge (host-side)

Because Arduino Mega 2560 (AVR) can't run micro-ROS reliably, this repo provides a lightweight ROS 2 Python node that translates topics to the serial protocol.

Location: `src/wheel_control/wheel_control/serial_motor_bridge.py`

Subscriptions:
- `/motors` (std_msgs/Int16MultiArray): data = [left, right] in [-255, 255]
- `/cmd_vel` (geometry_msgs/Twist) [optional]: maps linear/angular to left/right PWM

Parameters:
- `port` (string, default `/dev/ttyACM0`)
- `baud` (int, default 115200)
- `watchdog_ms` (int, default 600)
- `use_cmd_vel` (bool, default true)
- `wheel_separation` (float m, default 0.40)
- `max_speed_mps` (float m/s that equals |PWM|=255, default 1.0)

Run prerequisites:
- A working ROS 2 Humble environment on the host
- Python dependencies: `pyserial` (install with `pip install pyserial` if not present)

Build and run:
```bash
cd ~/control_one/wheel_control
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

# Run the bridge
ros2 run wheel_control serial_motor_bridge --ros-args -p port:=/dev/ttyACM0 -p use_cmd_vel:=true
```

Publish motor commands (example):
```
# Left reverse 120, Right forward 200
ros2 topic pub /motors std_msgs/Int16MultiArray '{data: [-120, 200]}' -1
```

Use cmd_vel (example):
```
# 0.3 m/s forward
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.3}, angular: {z: 0.0}}' -1
```

Notes:
- If your robot moves backwards, flip the `INVERT_M1`/`INVERT_M2` flags in the Arduino sketch or swap motor leads.
- Adjust `wheel_separation` and `max_speed_mps` to match your platform.

## Safety

- Always lift wheels or use a stand when first testing.
- Ensure a common ground between Arduino and driver.
- The firmware includes a watchdog that stops motors if commands stop arriving.

## License

MIT

## Teleop (keyboard)

You can drive the robot from the keyboard using a simple teleop compatible with the above bridge.

Location: `src/wheel_control/wheel_control/teleop_keyboard.py`

Default mode publishes `/cmd_vel`:
```bash
source install/setup.bash
ros2 run wheel_control teleop_keyboard
```

PWM mode publishes `/motors` directly:
```bash
source install/setup.bash
ros2 run wheel_control teleop_keyboard --ros-args -p mode:=motors -p topic:=/motors
```

Key bindings (similar to teleop_twist_keyboard):

```
	Movement
		u    i    o
		j    k    l
		m    ,    .

	i: forward, ,: backward, j: rotate left, l: rotate right
	u/o: forward + rotate  |  m/.: backward + rotate
	k or space: stop

	Speed controls:
	q/z : increase/decrease both linear and angular scales
	w/x : increase/decrease linear only
	e/c : increase/decrease angular only
```

Tips:
- Twist mode uses parameters `lin` (m/s) and `ang` (rad/s) as base scales; adjust with q/z/w/x/e/c.
- Motors mode uses `pwm_step` (default 25) per key press and clamps to `pwm_max` (default 255).

## ROS 2 launch: serial bridge + joystick (F310)

This package includes a launch file that starts:
- The serial bridge to the Arduino at 115200 baud
- The joystick driver (`joy`) and `teleop_twist_joy` configured for Logitech F310

Build and launch:
```bash
cd ~/control_one/wheel_control
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch wheel_control bringup.launch.py port:=/dev/ttyACM0 baud:=115200 joy_dev:=/dev/input/js0
```

Parameters you can override:
- `port` (default `/dev/ttyACM0`), `baud` (default `115200`)
- `use_cmd_vel` (default `true`), `wheel_separation` (0.40), `max_speed_mps` (1.0)
- `joy_dev` (`/dev/input/js0`), `joy_deadzone` (0.05), `joy_autorepeat_rate` (20.0)

F310 notes:
- The config at `src/wheel_control/config/teleop_f310.yaml` maps left stick vertical to forward/back and left stick horizontal to yaw. Hold LB to enable motion; RB is turbo.

## RealSense navigation stack (realsense_nav)

This workspace also contains a small perception + navigation stack that uses a RealSense camera, a segmentation model (SegFormer), YOLO-based scene-graph, and a pure-pursuit controller for local navigation.

Key components
- `scene_graph_node` (realsense_nav): builds a compact JSON scene graph from YOLO detections or segmentation masks and publishes on `/scene_graph`.
- `segformer` (segmentation package): SegFormer-based semantic segmentation (used instead of the older color-based segmentation).
- `llm_goal_detection` (realsense_nav): resolve natural-language goals (`/llm_goal`) into a concrete target from `/scene_graph` and publish `/topo_goal` and `/goal/position` for the planner.
- `pure_pursuit_controller` (realsense_nav): subscribes to `/goal/position` and `/goal/detected`, computes a local trajectory and publishes `/cmd_vel` for motor control.
- `serial_motor_bridge` (wheel_control): translates `/cmd_vel` into Arduino motor commands (already documented above).
- `view_segmentation` (realsense_nav): an OpenCV viewer that displays segmentation, trajectory overlay and a YOLO overlay panel (useful for debugging visual goals).

Launch (full stack)
The repository includes a single `full_navigation.launch.py` that brings up the camera, SegFormer segmentation, scene graph, LLM goal resolver, controller and serial bridge.

Build and run the full navigation stack:
```bash
cd ~/control_one/wheel_control
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 launch realsense_nav full_navigation.launch.py
```

Set a goal to the detected chair (example)
- Publish a short natural-language command; `llm_goal_detection` will resolve it and publish a topo/goals for the planner:
```bash
ros2 topic pub /llm_goal std_msgs/msg/String "data: 'go to chair'" -1
```
- Verify the resolved target and controller output:
```bash
ros2 topic echo /topo_goal --once
ros2 topic echo /goal/position --once
ros2 topic echo /cmd_vel
```

Notes
- The stack now uses SegFormer (deep model) for semantic segmentation — color-based segmentation is no longer used by default. If you want to switch back to a simple color-based pipeline, edit the launch file or use the older `segmentation_node`.
- If two goal sources exist (e.g., the legacy `goal_detection_node` detecting a yellow cone and `llm_goal_detection`), the most recent `/goal/position` or `/topo_goal` will be used by the controller; remove or disable the legacy node in the launch if you want the LLM goal to be authoritative.
- `view_segmentation` shows an additional YOLO overlay panel (label + confidence) when `/scene_graph` contains detections.

Want me to add parameter remaps or tweak the launch to disable the old cone detector by default? I can update the launch file for you.