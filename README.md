# Turtle Controller Package

**A Complete ROS2 Communication Example**

This package demonstrates **all major ROS2 communication patterns** in a single practical application:

1. **Publisher** - Send velocity commands
2. **Subscriber** - Receive pose updates
3. **Service Server** - Provide toggle service (with custom interface)
4. **Service Client** - Call SetPen service
5. **Action Server** - Execute turn commands with feedback

Controls a turtlesim turtle with autonomous random motion, wall detection, dynamic pen colors, and controllable turn actions.

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

**Usage Example**:
```bash
# Activate the turtle
ros2 service call /turtle_state_toggle_service my_robot_interfaces/srv/ToggleTurtleState "{turtle_switch: true}"

# Deactivate the turtle
ros2 service call /turtle_state_toggle_service my_robot_interfaces/srv/ToggleTurtleState "{turtle_switch: false}"
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

### 5. **Action Server** - Turn Turtle with Feedback (Custom Interface)

**Implementation**:
```python
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup

self.turn_turtle_srv = ActionServer(
    self, 
    TurnTurtle, 
    "turn_turtle_srv",
    goal_callback=self.goal_cb,
    execute_callback=self.execute_cb,
    cancel_callback=self.cancel_cb,
    callback_group=ReentrantCallbackGroup()
)

def goal_cb(self, goal_request):
    """Decide whether to accept or reject the goal"""
    if not self.is_active:
        return GoalResponse.REJECT
    return GoalResponse.ACCEPT

def execute_cb(self, goal_handle):
    """Execute the turn action with feedback"""
    self.action_active = True
    deg_to_turn = goal_handle.request.turn_degree
    
    # Set action command (published by control_loop)
    self.action_cmdVel.angular.z = 1.57  # rad/s
    
    # Provide feedback during execution
    feedback = TurnTurtle.Feedback()
    while not_complete:
        feedback.progress = calculate_progress()
        goal_handle.publish_feedback(feedback)
    
    self.action_active = False
    goal_handle.succeed()
    return result

def cancel_cb(self, goal_handle):
    """Handle cancellation requests"""
    return CancelResponse.ACCEPT
```

- **Action**: `/turn_turtle_srv`
- **Type**: `my_robot_interfaces/action/TurnTurtle` (**Custom Interface**)
- **Purpose**: Turn turtle by specified degrees with real-time feedback

#### Custom Action Interface: TurnTurtle

**Location**: `my_robot_interfaces/action/TurnTurtle.action`

**Definition**:
```
# Goal
float32 turn_degree    # Degrees to turn (positive = counter-clockwise)
---
# Result
float32 heading        # Final orientation in radians
---
# Feedback
float32 current_angle   # Current angle in degrees
float32 remaining_angle # Remaining angle to turn
float32 progress        # Percentage complete (0-100)
```

**Why Actions?**
Actions are ideal for **long-running tasks** that need:
- **Feedback**: Progress updates during execution
- **Cancellation**: Ability to stop mid-execution
- **Result**: Final outcome after completion

Unlike services (which block until complete), actions provide real-time status updates.

**Usage Examples**:
```bash
# Turn 180 degrees with feedback
ros2 action send_goal /turn_turtle_srv my_robot_interfaces/action/TurnTurtle "{turn_degree: 180.0}" --feedback

# Turn 90 degrees counter-clockwise
ros2 action send_goal /turn_turtle_srv my_robot_interfaces/action/TurnTurtle "{turn_degree: 90.0}" --feedback

# Turn 90 degrees clockwise (negative)
ros2 action send_goal /turn_turtle_srv my_robot_interfaces/action/TurnTurtle "{turn_degree: -90.0}" --feedback

# Cancel during execution (Ctrl+C while action is running)
```

**Key Features**:
- **Goal Callback**: Validates and accepts/rejects goals
- **Execute Callback**: Performs the turn with feedback loop
- **Cancel Callback**: Handles cancellation requests
- **Non-blocking**: Uses `MultiThreadedExecutor` for concurrent execution
- **Priority System**: Action commands override normal motion

---

## 📦 Installation

```bash
# Clone repository
cd ~/ros2_ws/src
git clone <repository-url> turtle_controller

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build (important: build interfaces first!)
colcon build --packages-select my_robot_interfaces
colcon build --packages-select turtle_controller
source install/setup.bash
```

---

## 🚀 Quick Start

```bash
# Terminal 1: Start turtlesim
ros2 run turtlesim turtlesim_node

# Terminal 2: Run controller
ros2 run turtle_controller turtle_ctrl

# Terminal 3: Test service (optional)
ros2 service call /turtle_state_toggle_service my_robot_interfaces/srv/ToggleTurtleState "{turtle_switch: true}"

# Terminal 4: Test action
ros2 action send_goal /turn_turtle_srv my_robot_interfaces/action/TurnTurtle "{turn_degree: 180.0}" --feedback
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

**Action Server**: Send turn command
```bash
ros2 action send_goal /turn_turtle_srv my_robot_interfaces/action/TurnTurtle "{turn_degree: 180.0}" --feedback
```

---

## 🎨 Application Features

- **Random Motion**: Smooth movement with configurable motion patterns (Gaussian, Cauchy, etc.)
- **Wall Detection**: Boundaries at x:[1,10], y:[1,10]
- **Wall Avoidance**: Automatic turning when approaching walls
- **Dynamic Pen Colors**: Visual feedback (blue=normal, red=wall proximity)
- **Toggle Control**: Enable/disable motion via service
- **Precise Turns**: Execute specific angle rotations via action server
- **Real-time Feedback**: Progress updates during action execution

---

## 🏗️ Architecture: Action vs Normal Motion

### **Control Flow with Action Priority**

```
┌─────────────────────────────────────────────────────────┐
│         MultiThreadedExecutor                           │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  Thread 1: control_loop() (0.1s timer)                  │
│  ┌────────────────────────────────────────────┐         │
│  │ if action_active:                          │         │
│  │     publish(action_cmdVel)  ◄──────────────┼────┐    │
│  │ else:                                      │    │    │
│  │     publish(normal_cmdVel)                 │    │    │
│  └────────────────────────────────────────────┘    │    │
│                                                    │    │
│  Thread 2: execute_cb() (action server)            │    │
│  ┌────────────────────────────────────────────┐    │    │
│  │ action_active = True                       │    │    │
│  │ action_cmdVel.angular.z = 1.57 ────────────┼────┘    │
│  │ while turning:                             │         │
│  │     publish feedback                       │         │
│  │     check cancellation                     │         │
│  │ action_active = False                      │         │
│  └────────────────────────────────────────────┘         │
└─────────────────────────────────────────────────────────┘
```

**Key Points**:
- **Single publishing point**: Only `control_loop()` publishes to `/cmd_vel`
- **Action priority**: When `action_active = True`, action commands override normal motion
- **Seamless transition**: Automatically returns to normal motion when action completes
- **Concurrent execution**: Action feedback runs while control loop continues

---

## 🐛 Troubleshooting

**Turtle doesn't move**: Activate it via service
```bash
ros2 service call /turtle_state_toggle_service my_robot_interfaces/srv/ToggleTurtleState "{turtle_switch: true}"
```

**Action/Service not found**: Rebuild custom interface package
```bash
cd ~/ros2_ws
rm -rf build/my_robot_interfaces install/my_robot_interfaces
colcon build --packages-select my_robot_interfaces
source install/setup.bash
```

**Action feedback assertion error**: Make sure action definition uses `float32` types and workspace is sourced
```bash
ros2 interface show my_robot_interfaces/action/TurnTurtle  # Verify types
source ~/ros2_ws/install/setup.bash  # Source in all terminals
```

**Topics/services missing**: Check running nodes
```bash
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
```

---

## 📊 Communication Pattern Summary

| Pattern | Name | Topic/Service/Action | Type | Purpose |
|---------|------|---------------------|------|---------|
| Publisher | Velocity Control | `/turtle1/cmd_vel` | `Twist` | Send movement commands |
| Subscriber | Pose Monitor | `/turtle1/pose` | `Pose` | Receive position updates |
| Service Server | Toggle Control | `/turtle_state_toggle_service` | `ToggleTurtleState` | Enable/disable motion |
| Service Client | Pen Control | `/turtle1/set_pen` | `SetPen` | Change pen properties |
| Action Server | Turn Command | `/turn_turtle_srv` | `TurnTurtle` | Execute turns with feedback |

---
## 👨‍💻 Author

Xafaroth

## 🤝 Contributing

Educational contributions welcome! This package is designed as a learning resource for ROS2 communication patterns.