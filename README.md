# Universal Robots ROS2 Driver

Alpha version of the ROS2 Universal Robots driver. Should be transferred to the Universal Robots org when ready.

This branch is prepared for use with [Moveit!](https://moveit.ros.org/)

## General driver information
Driver currently only supports position joint interface which means only position-based controllers can be used with 
the ROS2 driver. [Universal Robots Client Library](https://github.com/UniversalRobots/Universal_Robots_Client_Library) includes also
velocity-based control whose support will be addressed in additional development of ROS2 driver.

## Requirements

Follow the [instructions](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver#setting-up-a-ur-robot-for-ur_robot_driver) in the paragraph 
[`Prepare the robot` ](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver#prepare-the-robot)

## Build Instructions

To build this package follow the [instructions](https://index.ros.org/doc/ros2/Tutorials/Workspace/Creating-A-Workspace/) for installation of ROS2.

After installation create a ROS Foxy workspace:
```
cd $HOME
mkdir -p ws_driver/src
cd ws_driver
source /opt/ros/$ROS_DISTRO/setup.bash

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

```

Clone this repo into the `src/` directory, then:

```
# Clone source-based dependencies into src/ directory
vcs import --skip-existing --input src/Universal_Robots_ROS2_Driver/.repos.yaml src

# Install package-based dependencies
rosdep install -y --rosdistro $ROS_DISTRO --ignore-src --from-paths src

# Build sources
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```
Source the workspace:

```
cd ~/ws_driver
source install/setup.bash
```

Start the driver with Moveit!:

```
ros2 launch ur_ros2_control_demos ur5_e_run_move_group.launch.py
```

Start the `joint_state_controller`:

```
ros2 control load_start_controller joint_state_controller
```

Start the `ur_joint_trajectory_controller`:

```
ros2 control load_start_controller ur_joint_trajectory_controller
```

## Testing
Use the Moveit rviz plugin to plan and execute the trajectory. Execute trajectories with
lower values of speed scaling factor set on teach pedant (e.g. 0.5).
Trajectory execution will be improved once `ScalingTrajectoryControllers` are implemented.

