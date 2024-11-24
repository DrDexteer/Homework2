
# ros2_kdl_package

## :package: About

This package contains the tutorial code to create and run your C++ node using KDL.

Created following [ROS 2 Documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html).

## :hammer: Build
Clone this package in the `src` folder of your ROS 2 workspace. Check for missing dependencies:
```bash
$ rosdep install -i --from-path src --rosdistro humble -y
```
Build your new package:
```bash
$ colcon build --packages-select ros2_kdl_package
```
Source the setup files:
```bash
$ . install/setup.bash
```

## :white_check_mark: Usage
### Running the Node
Run the node:
```bash
$ ros2 run ros2_kdl_package ros2_kdl_node
```

By default, the node publishes joint position commands. 

### Using Velocity Commands
To use velocity commands:
```bash
$ ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=velocity
```
In this case, the robot must be launched with the velocity interface:
```bash
$ ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller"
```

### Using Effort Commands
To use effort commands:
```bash
$ ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=effort -p traj_type:=linear_polynomial -p control_type:=joint_space
```
In this case, the robot must be launched with the effort interface:
```bash
$ ros2 launch iiwa_bringup iiwa.launch.py use_sim:="true" command_interface:="effort" robot_controller:="effort_controller"
```

---

## :chart_with_upwards_trend: Options

### Trajectory Types
Specify the desired trajectory type using the `traj_type` parameter:
- **Linear Polynomial**:
  ```bash
  --ros-args -p traj_type:=linear_polynomial
  ```
  Generates a linear trajectory using a cubic polynomial velocity profile.
  
- **Linear Trapezoidal**:
  ```bash
  --ros-args -p traj_type:=linear_trapezoidal
  ```
  Generates a linear trajectory using a trapezoidal velocity profile.

- **Circle Polynomial**:
  ```bash
  --ros-args -p traj_type:=circle_polynomial
  ```
  Generates a circular trajectory using a cubic polynomial velocity profile.

- **Circle Trapezoidal**:
  ```bash
  --ros-args -p traj_type:=circle_trapezoidal
  ```
  Generates a circular trajectory using a trapezoidal velocity profile.

### Control Types
Specify the desired control type using the `control_type` parameter:
- **Joint Space Control**:
  ```bash
  --ros-args -p control_type:=joint_space
  ```
  Executes the desired trajectory in joint space. Utilizes the `idCntr` function for computing joint torques.

- **Operational Space Control**:
  ```bash
  --ros-args -p control_type:=operative_space
  ```
  Executes the desired trajectory in Cartesian space. Utilizes the `idCntr2` function for computing torques based on end-effector position, velocity, and acceleration.

### Combined Example
For an effort-based control of a circular trajectory with a trapezoidal profile in operational space:
```bash
$ ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=effort -p traj_type:=circle_trapezoidal -p control_type:=operative_space
```

For a joint-space control of a linear trajectory with a cubic polynomial profile:
```bash
$ ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=effort -p traj_type:=linear_polynomial -p control_type:=joint_space
```

---

## :clipboard: Notes
- Make sure the correct interface (`position`, `velocity`, or `effort`) is set in both the launch command and the node parameters.
- Ensure all required dependencies are installed and the robot's configuration matches the specified command and trajectory types.

--- 
