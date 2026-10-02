This is a fork from https://github.com/coen132/Arctos adapted for ROS 2 Jazzy.

## Jazzy simulation (recommended)

`arctos_moveit_config` is the canonical simulation configuration. The
`arctos_bringup` and `arctos_moveit` demo launch commands use this same configuration.
The model uses one `mock_components/GenericSystem`, the standard arm and gripper
`joint_trajectory_controller` instances, and a `joint_state_broadcaster`.
This is kinematic simulation: position commands produce simulated joint feedback;
it does not simulate gravity, contact forces, motor electronics, or payload dynamics.

```bash
cd ~/arctos_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch arctos_moveit_config demo.launch.py
```

In RViz select planning group `arm`, set a reachable goal, then use **Plan** or
**Plan & Execute**. The `gripper` group supports joint-space goals. Set
`use_rviz:=false` for headless operation.

### Repeatable headless validation

```bash
ros2 run arctos_moveit_config validate_simulation.py
```

This starts and stops its own headless demo in ROS domain 173. Use
`--domain-id NUMBER` to select another unused domain and `--log PATH` to change
the launch log location. It exits nonzero on failure and checks controller
activation, all eight joint states, initial collision validity, end-effector
FK/IK, timed arm/gripper plans, execution results, and final joint feedback.

### Obstacle avoidance demo

```bash
source /opt/ros/jazzy/setup.bash
source ~/arctos_ws/install/setup.bash
ros2 run arctos_moveit_config obstacle_demo.py
```

The demo launches its own mock simulation and RViz in ROS domain **174**; no
separate launch is needed. Choose another unused domain with `--domain-id NUMBER`.
It moves to a fixed start configuration, plans an unobstructed baseline, then
uses forward kinematics to place an 8 cm box across that path (falling back to
smaller boxes if necessary). The box is a real MoveIt planning-scene collision
object in the `world` frame.

The demo verifies that the box blocks the baseline while leaving both endpoints
collision-free, replans using OMPL, checks the detour at joint-space intervals
of at most 0.01 rad, and executes it. Final joint feedback must match the goal.
This is sampled collision validation, not a continuous collision guarantee.
Box placement and the detour can vary with the randomized planner.

RViz stays open with the box visible after execution; press Enter in the terminal
to close the simulation. For automated validation without a display:

```bash
ros2 run arctos_moveit_config obstacle_demo.py --headless
```

Headless mode exits after the demo and returns nonzero on failure. Both modes
stop their launched processes on exit. Launch output is saved to
`/tmp/arctos-obstacle-demo.log` (override with `--log PATH`).

### Model and limits

- Geometry comes from `arctos_description/urdf/arctos_urdf.xacro`. The demo disables
  its legacy control additions and includes exactly one mock control system.
- Joint naming is `joint_1` through `joint_6`, plus `jaw1` and `jaw2`.
- `arctos_moveit_config/config/joint_limits.yaml` contains **simulation tuning**:
  arm 0.5 rad/s and 1.0 rad/s²; jaws 0.01 m/s and 0.02 m/s². These are not
  manufacturer ratings. Default planning scaling is 10%.
- Arm joints retain the source model's continuous-joint assumptions. Physical
  travel limits and calibration have not been established for a real build.
- The URDF supplies the fixed world frame; the SRDF attaches the gripper to
  `Link_6_1`. Only the arm uses KDL; the branched gripper uses joint-space planning.

The older packages/configurations below remain as upstream references; use the
canonical demo above for the validated Jazzy simulation.

## Original upstream instructions (ROS 2 Humble)

# Arctos

This is a ROS2 package for the Arctos arm. make sure [ROS2](https://docs.ros.org/en/humble/Installation.html) and [MoveIt2](https://moveit.ros.org/install-moveit2/binary/) are installed, along with their dev tools.

## Installation
### Install dependency controller packages
`sudo apt install ros-humble-controller*`
`sudo apt-get install ros-humble-can-msgs`
`pip install catkin_pkg`

### Make sure everything is up to date
`sudo apt update` \
`sudo apt upgrade`

### Make a directory and clone the source code
`mkdir -p arctos/src`\
`cd arctos/src`\
`git clone https://github.com/coen132/arctos.git`

### Build the workspace
`cd ~/arctos`\
`colcon build --symlink-install`

This should run without any errors, otherwise make sure the dependencies are installed and sourced correctly.

## Sourcing the package in your bash script
After installation make sure to automatically source the package in your bash script, otherwise you need to do it everytime you run the package
Copy and paste the following at the bottom of the bash file: "source ~/arctos/install/local_setup.bash"

`sudo nano ~/.bashrc`

## Launching the simulation

To test out and make sure everything is correctly installed you can launch the demo simulation \
`cd ~/arctos`\
`source install/local_setup.bash`\
`ros2 launch arctos_moveit_config demo.launch.py` 

This should launch RVIZ and load the Arctos robot arm. 
