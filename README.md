# ros2_control_robot_dynamics

URDF-backed robot kinematics and dynamics support for ros2_control controllers.

This package provides the Pinocchio-based `RobotModel` library used by the compliant controller wrapper to compute end-effector pose, Jacobians, gravity, Coriolis terms, and mass matrices from the robot description. It also provides `RobotDescriptionLoader`, a ROS parameter helper for retrieving URDF XML from a local or remote `robot_description` parameter.

The public header is installed at:

```cpp
#include <ros2_control_robot_dynamics/robot_model.hpp>
```

Main project: https://github.com/smihael/compliant_controllers

## Build

From the workspace root:

```bash
source install/setup.bash
colcon build --packages-select ros2_control_robot_dynamics --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF --symlink-install
```
