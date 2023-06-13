# Teleop
In first terminal
```bash
ros2 run turtlesim turtlesim_node
```

In second terminal source Rust workspace and run
```bash
ros2 run turtlesim_demo turtle_teleop_key
```

# Mimic
In first terminal
```bash
ros2 run turtlesim turtlesim_node
```

In second terminal spawn second turtle and then start teleop process
```bash
ros2 service call /spawn turtlesim/srv/Spawn "{x: 8.0, y: 8.0, theta: 0.0, name: 'turtle2'}"
ros2 run turtlesim_demo turtle_teleop_key
```

In third terminal source Rust workspace and run
```bash
ros2 run turtlesim_demo mimic --ros-args -r input/pose:=turtle1/pose -r output/cmd_vel:=turtle2/cmd_vel
```

# Draw square
In first terminal
```bash
ros2 run turtlesim turtlesim_node
```

In second terminal source Rust workspace and run
```bash
ros2 run turtlesim_demo draw_square
```
