# Turtle Controller Package

**A Complete ROS2 Communication Example**

This package demonstrates **all four fundamental ROS2 communication patterns** in a single practical application:

1. **Publisher** - Send velocity commands
2. **Subscriber** - Receive pose updates
3. **Service Server** - Provide toggle service (with custom interface)
4. **Service Client** - Call SetPen service

Controls a turtlesim turtle with autonomous Gaussian motion, wall detection, and dynamic pen colors.

---

## 📚 ROS2 Communication Patterns

### 1. **Publisher** - Controlling Turtle Movement

**Implementation**:
```python
self.cmdVel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

myMsg = Twist()
myMsg.linear.x = 1.0
myMsg.angular.z = 0.5
self.cmdVel_pub.publish(myMsg)
```

- **Topic**: `/turtle1/cmd_vel`
- **Type**: `geometry_msgs/msg/Twist`
- **Purpose**: Controls turtle's linear and angular velocities

---

### 2. **Subscriber** - Receiving Turtle Position

**Implementation**:
```python
self.pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.poseSubscriber_cb, 10)

def poseSubscriber_cb(self, currentPose):
    # Access: currentPose.x, currentPose.y, currentPose.theta
    if self.isReachingWall(currentPose):
        # React to position
        pass
```

- **Topic**: `/turtle1/pose`
- **Type**: `turtlesim/msg/Pose`
- **Purpose**: Monitor turtle position for wall detection

---

### 3. **Service Server** - Toggle Turtle State (Custom Interface)

**Implementation**:
```python
self.turtle_toggle_srv = self.create_service(
    ToggleTurtleState, 
    "turtle_state_toggle_service", 
    self.turtleToggleService_cb
)

def turtleToggleService_cb(self, request, response):
    self.is_active = request.turtle_switch
    response.success = True
    response.turtle_status = 'Turtle is activated' if request.turtle_switch else 'Turtle is deactivated'
    return response
```

- **Service**: `/turtle_state_toggle_service`
- **Type**: `my_robot_interfaces/srv/ToggleTurtleState` (**Custom Interface**)
- **Purpose**: Enable/disable turtle motion remotely

#### Custom Service Interface: ToggleTurtleState

**Location**: `my_robot_interfaces/srv/ToggleTurtleState.srv`

**Definition**:
```srv
# Request
bool turtle_switch    # true = activate, false = deactivate
---
# Response
bool success          # true if operation succeeded
string turtle_status  # "Turtle is activated" or "Turtle is deactivated"
```

**Why a Custom Interface?**
This demonstrates how to create your own service types beyond ROS2's built-in services. Custom interfaces allow you to define exactly what data your service needs to send and receive.

**How to Create**:
1. Create `my_robot_interfaces` package
2. Add `.srv` file in `srv/` directory
3. Update `CMakeLists.txt` to generate the interface
4. Build and source the workspace

**Usage Example**:
```bash
# Activate the turtle
ros2 service call /turtle_state_toggle_service my_robot_interfaces/srv/ToggleTurtleState "{turtle_switch: true}"
# Response: success: true, turtle_status: 'Turtle is activated'

# Deactivate the turtle
ros2 service call /turtle_state_toggle_service my_robot_interfaces/srv/ToggleTurtleState "{turtle_switch: false}"
# Response: success: true, turtle_status: 'Turtle is deactivated'
```

---

### 4. **Service Client** - Changing Pen Properties

**Implementation**:
```python
self.mySetPenClient = self.create_client(SetPen, '/turtle1/set_pen')

def setPenService_call(self, r, g, b, width, penOff):
    while not self.mySetPenClient.wait_for_service(1.0):
        self.get_logger().info('Waiting for service.')
    
    request = SetPen.Request()
    request.r, request.g, request.b = r, g, b
    request.width, request.off = width, penOff
    
    future = self.mySetPenClient.call_async(request)
    future.add_done_callback(self.setPenClient_cb)

def setPenClient_cb(self, future):
    response = future.result()
```

- **Service**: `/turtle1/set_pen`
- **Type**: `turtlesim/srv/SetPen`
- **Purpose**: Change pen color based on turtle behavior
  - **Red pen** (255, 0, 0): Approaching walls
  - **Blue pen** (0, 255, 0): Normal navigation

---

## 📦 Installation

```bash
# Clone repository
cd ~/ros2_ws/src
git clone <repository-url> turtle_controller

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --packages-select turtle_controller my_robot_interfaces
source install/setup.bash
```

---

## 🚀 Quick Start

```bash
# Terminal 1: Start turtlesim
ros2 run turtlesim turtlesim_node

# Terminal 2: Run controller
ros2 run turtle_controller turtle_ctrl

# Terminal 3: Toggle turtle (optional)
ros2 service call /turtle_state_toggle_service my_robot_interfaces/srv/ToggleTurtleState "{turtle_switch: false}"
```

---

## 🧪 Testing Each Pattern

**Publisher**: Monitor velocity commands
```bash
ros2 topic echo /turtle1/cmd_vel
```

**Subscriber**: Monitor pose updates
```bash
ros2 topic echo /turtle1/pose
```

**Service Server**: Call the toggle service
```bash
ros2 service call /turtle_state_toggle_service my_robot_interfaces/srv/ToggleTurtleState "{turtle_switch: true}"
```

**Service Client**: Watch pen color change automatically when turtle approaches walls

---

## 🎨 Application Features

- **Gaussian Random Motion**: Smooth, natural movement (mean=1.0, std=0.3)
- **Wall Detection**: Boundaries at x:[1,10], y:[1,10]
- **Wall Avoidance**: Automatic turning when approaching walls
- **Dynamic Pen Colors**: Visual feedback (blue=normal, red=wall proximity)

---

## 🐛 Troubleshooting

**Turtle doesn't move**: Activate it via service
```bash
ros2 service call /turtle_state_toggle_service my_robot_interfaces/srv/ToggleTurtleState "{turtle_switch: true}"
```

**Service not found**: Build custom interface package
```bash
colcon build --packages-select my_robot_interfaces
source install/setup.bash
```

**Topics/services missing**: Check running nodes
```bash
ros2 node list
ros2 topic list
ros2 service list
```

---

## 📚 Resources

- [ROS2 Publishers/Subscribers Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [ROS2 Services/Clients Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html)
- [Custom ROS2 Interfaces](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html)

---

## 👨‍💻 Author

Xafaroth

## 🤝 Contributing

Educational contributions welcome!