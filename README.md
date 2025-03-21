

#  robot_model_analyzer

**robot_model_analyzer** is a ROS 2 C++ package that performs analysis on a robot's URDF model using [Pinocchio](https://github.com/stack-of-tasks/pinocchio). It retrieves the robot description from the parameter server or a topic, parses the URDF, and runs key computations including:
- Forward kinematics to the end-effector
- Jacobian calculation
- Collision detection

## Features

- ✅ Subscribes to `/robot_description` topic (e.g., from `robot_state_publisher`)
- ✅ Optionally fetches URDF from the parameter server
- ✅ Saves URDF to disk
- ✅ Parses the URDF using Pinocchio
- ✅ Computes:
  - End-effector transform
  - Frame Jacobian
  - Collision detection between links

---

##  Dependencies

- ROS 2 (tested with Humble)
- [Pinocchio](https://github.com/stack-of-tasks/pinocchio)
- [urdf_parser](https://github.com/ros/urdf_parser)
- [ament_index_cpp](https://github.com/ament/ament_index)

Make sure Pinocchio is installed and visible to your C++ build system.

---

##  Usage

### 1. Build the package

```bash
colcon build --packages-select robot_model_analyzer
source install/setup.bash
```

### 2. Run the node

Make sure your robot description is published by `robot_state_publisher` or another node. Then run:

```bash
ros2 run robot_model_analyzer robot_model_analyzer
```

### 3. Set the end-effector frame (optional)

You can specify the end-effector frame via a parameter:

```bash
ros2 run robot_model_analyzer robot_model_analyzer --ros-args -p ee_frame:=<your_ee_frame>
```

If not specified, it defaults to `ee_link`.

---

##  Output

The node logs and prints:

- The transform of the end-effector
- The Jacobian matrix at the end-effector
- Any collisions detected between model links

Example console output:

```
End-effector transform:
...
Jacobian at end-effector:
...
Collision detected between link1 and link2
```

---

##  How It Works

1. Subscribes to `/robot_description` topic (or gets parameter)
2. Parses the URDF using `urdf_parser`
3. Saves URDF as `temp.urdf` in a local directory
4. Loads the model using Pinocchio
5. Computes:
   - Forward kinematics to the specified frame
   - Frame Jacobian
   - Collision checks between all pairs

---

