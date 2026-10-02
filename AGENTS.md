# Arctos workspace

## Build and run

- Use ROS 2 **Jazzy**. `src/README.md` starts with current instructions; its later Humble section is upstream reference material.
- Run from the workspace root, sourcing the underlay before building and the overlay before running:
  ```bash
  source /opt/ros/jazzy/setup.bash
  rosdep install --from-paths src --ignore-src -r -y
  colcon build --symlink-install
  source install/setup.bash
  ```
- After an initial workspace build, rebuild a changed package with `colcon build --symlink-install --packages-select <package>` and source `install/setup.bash` again. `arctos_description` has a CMake dependency on `arctos_hardware_interface` even for mock simulation.
- Launch: `ros2 launch arctos_moveit_config demo.launch.py`; append `use_rviz:=false` for headless operation.
- `build/`, `install/`, and `log/` are ignored colcon outputs; edit sources under `src/`.

## Runtime sources of truth

- `src/arctos_moveit_config/` is the canonical simulation configuration. The `arctos_bringup` and `arctos_moveit` **demo** launchers delegate to it; similarly named configs in `arctos_moveit` and `arctos_description/config` are not the demo's configuration.
- Demo launch uses `MoveItConfigsBuilder("arctos_urdf", package_name="arctos_moveit_config")` and MoveIt's `generate_demo_launch`. Start there when tracing node startup.
- The canonical `config/arctos_urdf.urdf.xacro` includes `src/arctos_description/urdf/arctos_urdf.xacro` for geometry, disables `legacy_control`, and adds exactly one `mock_components/GenericSystem`. Preserve that override: the geometry file defaults to enabling legacy transmission/Gazebo/custom-control additions.
- `.setup_assistant` still points to `arctos_description/urdf/arctos.urdf`; the demo wrapper uses the Xacro above. Account for this mismatch before regenerating configs.
- Simulation is kinematic. Its limits in `arctos_moveit_config/config/joint_limits.yaml` are simulation tuning, not hardware ratings.
- Controlled joints are `joint_1`–`joint_6`, `jaw1`, and `jaw2`. Keep the control Xacro, initial positions, controller YAMLs, limits, and SRDF consistent. Both arm and gripper use `JointTrajectoryController`; only the arm has KDL IK, and the branched gripper uses joint-space goals.
- The geometry already defines `world -> base_link`; the SRDF end-effector parent is `Link_6_1`. Do not add a duplicate world virtual joint.
- `arctos_hardware_interface` builds the custom hardware/controller plugin library. `arctos_motor_driver` currently declares no compiled targets despite containing C++ sources; a successful build does not compile those files.

## Verification

- Primary planning/execution check (after building and sourcing):
  ```bash
  ros2 run arctos_moveit_config validate_simulation.py
  ```
  It starts and stops its own headless demo, checking controllers, eight-joint feedback, collision validity, FK/IK, timed plans, and arm/gripper execution.
- For obstacle/planning-scene changes:
  ```bash
  ros2 run arctos_moveit_config obstacle_demo.py --headless
  ```
  Without `--headless`, it opens RViz and waits for Enter. Collision validation samples the trajectory; OMPL results can vary.
- These scripts use isolated ROS domains **173** and **174**, respectively, with localhost discovery. Both accept `--domain-id NUMBER` and `--log PATH`; no separately launched demo is needed.
- These checks are installed scripts, not registered CTest tests. `colcon test` alone does not exercise simulation; existing `BUILD_TESTING` blocks only register ament lint checks.
