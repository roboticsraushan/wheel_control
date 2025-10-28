# RealSense Navigation System Architecture

```mermaid
graph TB
    subgraph Hardware
        RS[Intel RealSense D455<br/>RGB + Depth Camera]
        ARD[Arduino Due<br/>Motor Controller]
        MOT[Cytron Motor Driver<br/>Dual Channel]
        M1[Motor 1]
        M2[Motor 2]
        JOY[Logitech F310<br/>Gamepad]
    end

    subgraph ROS2_Nodes
        CAM[realsense2_camera_node<br/>Camera Driver]
        SEG[segmentation_node<br/>HSV Color Segmentation]
        BRIDGE[serial_motor_bridge<br/>Serial Communication]
        JOYNODE[joy_node<br/>Joystick Input]
        TELEOP[teleop_twist_joy<br/>Gamepad Mapping]
    end

    subgraph Topics
        RGB["(/camera/camera/color/image_raw)<br/>1280x720 30fps"]
        DEPTH["(/camera/camera/depth/image_rect_raw)<br/>848x480 30fps"]
        CMDVEL["(/cmd_vel)<br/>Twist: linear.x angular.z"]
        SEGIMG["(/segmentation/image)<br/>Visualization Overlay"]
        CENT["(/segmentation/centroid)<br/>Target Point x y"]
        NAV["(/segmentation/navigable)<br/>Boolean: Path Clear"]
        JOYCMD["(/joy)<br/>Gamepad State"]
    end

    subgraph Processing
        HSV["HSV Thresholding<br/>H:0-180 S:0-50 V:100-255"]
        MORPH["Morphological Ops<br/>CLOSE + OPEN"]
        CONT["Contour Detection<br/>Find Largest Area"]
        CENTR["Centroid Calculation<br/>Moments"]
        CTRL["Proportional Controller<br/>Error = centroid - center"]
    end

    RS -->|USB 3.2| CAM
    CAM --> RGB
    CAM --> DEPTH
    
    RGB --> SEG
    DEPTH --> SEG
    
    SEG --> HSV
    HSV --> MORPH
    MORPH --> CONT
    CONT --> CENTR
    CENTR --> CTRL
    
    CTRL --> CMDVEL
    SEG --> SEGIMG
    SEG --> CENT
    SEG --> NAV
    
    JOY -->|USB| JOYNODE
    JOYNODE --> JOYCMD
    JOYCMD --> TELEOP
    TELEOP --> CMDVEL
    
    CMDVEL --> BRIDGE
    BRIDGE -->|Serial 115200| ARD
    ARD -->|DIR+PWM| MOT
    MOT --> M1
    MOT --> M2

    
```

## Key Components

### 1. **Input Layer**
- **RealSense D455**: RGB (1280x720) + Depth (848x480) @ 30fps
- **Gamepad**: Logitech F310 for manual control

### 2. **Processing Layer**
- **Segmentation Pipeline**:
  - RGB → HSV conversion
  - Color thresholding (floor detection)
  - Morphological operations (noise removal)
  - Contour detection (find navigable area)
  - Centroid calculation (target point)
  
- **Control Algorithm**:
  - Proportional control: `angular = -gain × (centroid_x - center_x)`
  - Linear velocity: `linear = gain × (1 - |error|)`

### 3. **Output Layer**
- **Velocity Commands**: `/cmd_vel` (Twist messages)
- **Serial Bridge**: Converts Twist → Arduino commands
- **Motor Control**: DIR+PWM signals to Cytron driver

## Data Flow

```mermaid
sequenceDiagram
    participant Camera
    participant SegNode
    participant Controller
    participant Bridge
    participant Arduino
    participant Motors

    Camera->>SegNode: RGB Frame (30Hz)
    Camera->>SegNode: Depth Frame (30Hz)
    
    SegNode->>SegNode: HSV Segmentation
    SegNode->>SegNode: Find Centroid
    SegNode->>Controller: Centroid Position
    
    Controller->>Controller: Calculate Error
    Controller->>Controller: Proportional Control
    Controller->>Bridge: Twist(linear, angular)
    
    Bridge->>Bridge: Differential Drive Math
    Bridge->>Arduino: "M left right\n"
    
    Arduino->>Motors: PWM Signals
    Motors->>Motors: Robot Movement
```

## Navigation Logic

```mermaid
flowchart TD
    START([Camera Frame])
    HSV[Convert to HSV]
    THRESH[Apply Thresholds]
    MORPH[Morphological Ops]
    CONTOUR[Find Contours]
    CHECK{Area > Min?}
    CENT[Calculate Centroid]
    ERROR[Calculate Error]
    VEL[Generate Velocity]
    STOP[Send Stop Command]
    PUB[Publish to /cmd_vel]
    
    START --> HSV
    HSV --> THRESH
    THRESH --> MORPH
    MORPH --> CONTOUR
    CONTOUR --> CHECK
    CHECK -->|Yes| CENT
    CHECK -->|No| STOP
    CENT --> ERROR
    ERROR --> VEL
    VEL --> PUB
    STOP --> PUB
    
    style CHECK fill:#ffe1e1
    style VEL fill:#e1ffe1
    style STOP fill:#ffe1e1
```

## Control Equations

**Angular Velocity:**
```
error_x = (centroid_x - center_x) / center_x  # Normalized [-1, 1]
angular_vel = -angular_gain × error_x         # Negative for correct steering
angular_vel = clip(angular_vel, -max_angular, max_angular)
```

**Linear Velocity:**
```
linear_vel = linear_gain × (1 - |error_x|)    # Slow down when turning
linear_vel = clip(linear_vel, 0, max_linear)
```

**Differential Drive:**
```
v_left = linear - (wheel_sep/2) × angular
v_right = linear + (wheel_sep/2) × angular
pwm_left = 255 × (v_left / max_speed)
pwm_right = 255 × (v_right / max_speed)
```
