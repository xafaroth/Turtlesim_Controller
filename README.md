# Turtle Controller Package

A ROS2 package for controlling a turtlesim turtle with random Gaussian motion, wall detection, and dynamic pen color changes. This has examples of how to run ros2 subsriber, publisher, service and client. Additionally, also has example of user defined interfaces for server.

## Overview

This package provides an autonomous turtle controller that:
- Moves the turtle with random Gaussian-distributed velocities
- Detects wall proximity and performs avoidance maneuvers
- Changes pen color when approaching and leaving walls
- Can be toggled on/off via a custom service

## Features

### 1. **Random Gaussian Motion**
The turtle moves with velocities sampled from Gaussian (normal) distributions:
- **Linear velocity**: Gaussian distribution with configurable mean and standard deviation
- **Angular velocity**: Gaussian distribution for smooth, natural turning behavior
- Velocities are clipped to safe operational ranges

### 2. **Wall Detection & Avoidance**
- Detects when the turtle approaches arena boundaries (x: 1-10, y: 1-10)
- Automatically adjusts motion to turn away from walls
- Provides visual feedback via pen color changes

### 3. **Dynamic Pen Control**
- **Blue pen** (RGB: 0, 255, 0): Normal navigation
- **Red pen** (RGB: 255, 0, 0): Wall proximity detected
- Pen width increases when near walls for visual emphasis

### 4. **Toggle Service**
Custom service to enable/disable turtle motion without stopping the node

## Package Structure

```
turtle_controller/
├── turtle_controller/
│   ├── __init__.py
│   └── controller.py          # Main controller node
├── package.xml
├── setup.py
└── README.md
```

## Dependencies

### ROS2 Packages
- `rclpy` - ROS2 Python client library
- `geometry_msgs` - Twist messages for velocity commands
- `turtlesim` - Turtle simulation and Pose/SetPen services
- `my_robot_interfaces` - Custom service definitions

### Python Packages
- `numpy` - For Gaussian random number generation

## Installation

1. **Clone the repository** into your ROS2 workspace:
```bash
cd ~/ros2_ws/src
git clone <repository-url> turtle_controller
```

2. **Install dependencies**:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. **Build the package**:
```bash
cd ~/ros2_ws
colcon build --packages-select turtle_controller
```

4. **Source the workspace**:
```bash
source ~/ros2_ws/install/setup.bash
```

## Usage

### 1. Start Turtlesim
```bash
ros2 run turtlesim turtlesim_node
```

### 2. Run the Turtle Controller
```bash
ros2 run turtle_controller turtle_ctrl
```

### 3. Toggle Turtle State (Optional)
In a new terminal, use the service to activate the turtle:
```bash
# Activate turtle
ros2 service call /turtle_state_toggle_service my_robot_interfaces/srv/ToggleTurtleState "{turtle_switch: true}"

# Deactivate turtle
ros2 service call /turtle_state_toggle_service my_robot_interfaces/srv/ToggleTurtleState "{turtle_switch: false}"
```

## Node Details

### Node Name
`turtle_controller`

### Published Topics
| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/turtle1/cmd_vel` | `geometry_msgs/msg/Twist` | Velocity commands for the turtle |

### Subscribed Topics
| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/turtle1/pose` | `turtlesim/msg/Pose` | Current pose of the turtle |

### Services

#### Server
| Service | Service Type | Description |
|---------|--------------|-------------|
| `/turtle_state_toggle_service` | `my_robot_interfaces/srv/ToggleTurtleState` | Enable/disable turtle motion |

#### Client
| Service | Service Type | Description |
|---------|--------------|-------------|
| `/turtle1/set_pen` | `turtlesim/srv/SetPen` | Change pen color and properties |

### Parameters

The controller uses the following motion parameters (can be modified in code):

```python
# Gaussian motion parameters
mean_linear = 1.0        # Mean linear velocity (m/s)
std_linear = 0.3         # Standard deviation for linear velocity
mean_angular = 0.0       # Mean angular velocity (rad/s)
std_angular = 0.5        # Standard deviation for angular velocity

# Wall detection thresholds
wall_threshold_min = 1.0  # Minimum x/y position
wall_threshold_max = 10.0 # Maximum x/y position

# Wall avoidance behavior
wall_linear_vel = 1.0     # Linear velocity when avoiding wall
wall_angular_vel = 1.5    # Angular velocity when avoiding wall
```

## How It Works

### Motion Generation
The controller generates motion using NumPy's Gaussian random number generator:

```python
# Generate Gaussian-distributed velocities
linear_vel = np.random.normal(mean_linear, std_linear)
angular_vel = np.random.normal(mean_angular, std_angular)

# Clip to safe ranges
linear_vel = np.clip(linear_vel, 0.1, 2.0)
angular_vel = np.clip(angular_vel, -2.0, 2.0)
```

### State Machine
1. **Idle State**: `is_active = False`
   - Turtle does not move
   - Waiting for activation via service

2. **Active State**: `is_active = True`
   - Normal navigation with Gaussian random motion
   - Wall detection active
   - Pen color changes based on proximity to walls

### Wall Detection Algorithm
```python
def isReachingWall(self, pose):
    if pose.x > 10 or pose.x < 1 or pose.y > 10 or pose.y < 1:
        return True
    return False
```

### Pen Color Logic
- **Entering wall zone**: Previous pose was safe AND current pose is near wall → Red pen
- **Leaving wall zone**: Previous pose was near wall AND current pose is safe → Blue pen

## Customization

### Adjust Gaussian Parameters
Modify the motion distribution in `controller.py`:

```python
# More conservative motion (lower variance)
linear_vel = np.random.normal(0.8, 0.2)
angular_vel = np.random.normal(0.0, 0.3)

# More aggressive motion (higher variance)
linear_vel = np.random.normal(1.5, 0.5)
angular_vel = np.random.normal(0.0, 0.8)
```

### Change Pen Colors
Modify the RGB values in the `call_set_pen_server()` calls:

```python
# Change red to orange (255, 165, 0)
self.call_set_pen_server(255, 165, 0, 2, 0)

# Change blue to green (0, 255, 0)
self.call_set_pen_server(0, 255, 0, 1, 0)
```

### Adjust Wall Thresholds
Modify the boundary conditions in `isReachingWall()`:

```python
# Tighter boundaries (more cautious)
if pose.x > 9 or pose.x < 2 or pose.y > 9 or pose.y < 2:
    return True

# Looser boundaries (less cautious)
if pose.x > 10.5 or pose.x < 0.5 or pose.y > 10.5 or pose.y < 0.5:
    return True
```

## Troubleshooting

### Turtle doesn't move
- Ensure the turtle is activated: Call the toggle service with `turtle_switch: true`
- Check that turtlesim_node is running
- Verify topics are connected: `ros2 topic list`

### Service not found
- Build and source the `my_robot_interfaces` package first
- Verify service is available: `ros2 service list`

### Pen color doesn't change
- Check that `/turtle1/set_pen` service is available
- Verify log messages show "changing pen color to..."
- Ensure turtlesim window is visible

### Erratic motion
- Adjust Gaussian distribution parameters (reduce standard deviation)
- Increase velocity clipping constraints
- Check pose subscription frequency

## Mathematical Background

### Gaussian Distribution
The motion uses the normal distribution: **N(μ, σ²)**

Where:
- **μ (mu)**: Mean value (center of distribution)
- **σ (sigma)**: Standard deviation (spread of distribution)

**Probability Density Function**:
```
f(x) = (1 / (σ√(2π))) * e^(-(x-μ)²/(2σ²))
```

**Properties**:
- ~68% of values within ±1σ of mean
- ~95% of values within ±2σ of mean
- Produces smooth, natural-looking motion

## Future Enhancements

- [ ] Add obstacle detection using laser scan
- [ ] Implement path planning algorithms
- [ ] Add multiple turtle coordination
- [ ] Create configurable parameters via ROS2 parameters
- [ ] Add dynamic reconfigure for real-time tuning
- [ ] Implement different motion patterns (Brownian, Lévy flight, etc.)
- [ ] Add visualization of trajectory statistics

## License

[Specify your license here]

## Author

[Your name/organization]

## Contributing

Contributions are welcome! Please submit pull requests or open issues for bugs and feature requests.

## Acknowledgments

- ROS2 Tutorials and Documentation
- Turtlesim package maintainers
- NumPy community
