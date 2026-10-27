# ROS 2 Turtlesim Control System

## Project Overview
A ROS 2 project implementing turtle movement control, custom messages, services, actions, parameters, and launch files using Turtlesim.


## Packages

- **`mechatronics_interfaces`**: Custom message (`TargetPose`), services (`SetSpeed`), and actions (`GoToPose`, `DrawSquare`).
- **`mechatronics_control`**: Implements turtle movement, speed control, reset functionality, square drawing, and the system launch file.

## Requirements
- Ubuntu 22.04
- ROS 2 Humble
- Turtlesim

## Build and Run

Source ROS 2 and build the packages:

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-up-to mechatronics_control
source install/setup.bash
```

Launch the system:

```bash
ros2 launch mechatronics_control system.launch.py
```

## Testing

Useful commands for inspecting the system:

```bash
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
ros2 interface list
```

The project supports:
- Moving the turtle to a target pose.
- Changing maximum linear and angular speeds.
- Resetting the turtle.
- Drawing a square using an Action Server.

## Bonus: ROS Bag

Turtle motion was recorded and replayed using ROS 2 Bags.

```bash
ros2 bag info square_run2
ros2 bag play square_run2
```

Screenshots documenting the Bag information and playback are included in the repository.

