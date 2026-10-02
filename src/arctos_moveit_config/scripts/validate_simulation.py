#!/usr/bin/env python3
"""Launch an isolated mock demo, validate planning/execution, and stop it."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.action import ActionClient
from controller_manager_msgs.srv import ListControllers
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan, GetStateValidity, GetPositionFK, GetPositionIK
from sensor_msgs.msg import JointState


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain-id', type=int, default=173,
                        help='Unused ROS domain for the isolated test')
    parser.add_argument('--log', default='/tmp/arctos-simulation-validation.log')
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    os.environ.pop('ROS_LOCALHOST_ONLY', None)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    log_path = Path(args.log)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('arctos_simulation_validation')
    latest = {}

    def update(msg):
        latest['msg'] = msg
        latest['received'] = time.monotonic()

    subscription = node.create_subscription(JointState, '/joint_states', update, 10)

    def wait(future, timeout=20):
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError('ROS request timed out')
        return future.result()

    def client(kind, name):
        result = node.create_client(kind, name)
        if not result.wait_for_service(timeout_sec=40):
            raise RuntimeError(f'Service unavailable: {name}')
        return result

    with log_path.open('w') as log:
        launch = subprocess.Popen(
            ['ros2', 'launch', 'arctos_moveit_config', 'demo.launch.py', 'use_rviz:=false'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            controllers = client(ListControllers, '/controller_manager/list_controllers')
            deadline = time.monotonic() + 45
            expected = {'arm_controller', 'gripper_controller', 'joint_state_broadcaster'}
            while time.monotonic() < deadline:
                active = {c.name for c in wait(controllers.call_async(ListControllers.Request())).controller
                          if c.state == 'active'}
                if expected <= active and 'msg' in latest:
                    break
                rclpy.spin_once(node, timeout_sec=0.2)
            else:
                raise RuntimeError(f'Controllers/joint states not ready: {active}')
            print('PASS: all three controllers active', flush=True)
            names = {f'joint_{i}' for i in range(1, 7)} | {'jaw1', 'jaw2'}
            assert names <= set(latest['msg'].name), 'Missing joint feedback'
            assert time.monotonic() - latest['received'] < 2, 'Stale joint feedback'
            validity = client(GetStateValidity, '/check_state_validity')
            request = GetStateValidity.Request()
            request.group_name = 'arm'
            request.robot_state.joint_state = latest['msg']
            assert wait(validity.call_async(request)).valid, 'Initial state is in collision'
            print('PASS: fresh eight-joint feedback and collision-free initial state', flush=True)
            planner = client(GetMotionPlan, '/plan_kinematic_path')
            fk = client(GetPositionFK, '/compute_fk')
            ik = client(GetPositionIK, '/compute_ik')
            fk_request = GetPositionFK.Request()
            fk_request.header.frame_id = 'world'
            fk_request.fk_link_names = ['Link_6_1']
            fk_request.robot_state.joint_state.name = [f'joint_{i}' for i in range(1, 7)]
            fk_request.robot_state.joint_state.position = [0.15, 0.0, 0.0, 0.0, 0.0, 0.0]
            fk_result = wait(fk.call_async(fk_request))
            assert fk_result.error_code.val == MoveItErrorCodes.SUCCESS, 'Forward kinematics failed'
            ik_request = GetPositionIK.Request()
            ik_request.ik_request.group_name = 'arm'
            ik_request.ik_request.ik_link_name = 'Link_6_1'
            ik_request.ik_request.robot_state.joint_state = latest['msg']
            ik_request.ik_request.pose_stamped = fk_result.pose_stamped[0]
            ik_request.ik_request.avoid_collisions = True
            ik_request.ik_request.timeout.sec = 2
            ik_result = wait(ik.call_async(ik_request))
            assert ik_result.error_code.val == MoveItErrorCodes.SUCCESS, 'End-effector IK failed'
            print('PASS: world-frame end-effector FK and collision-aware IK', flush=True)
            executor = ActionClient(node, ExecuteTrajectory, '/execute_trajectory')
            assert executor.wait_for_server(timeout_sec=20), 'Execution action unavailable'
            arm = {f'joint_{i}': 0.0 for i in range(1, 7)}
            for group, target in [('arm', dict(arm, joint_1=0.15)),
                                  ('arm', arm),
                                  ('gripper', {'jaw1': 0.005, 'jaw2': 0.005}),
                                  ('gripper', {'jaw1': 0.0, 'jaw2': 0.0})]:
                request = GetMotionPlan.Request()
                motion = request.motion_plan_request
                motion.group_name = group
                motion.pipeline_id = 'ompl'
                motion.allowed_planning_time = 5.0
                motion.num_planning_attempts = 3
                motion.max_velocity_scaling_factor = 0.5
                motion.max_acceleration_scaling_factor = 0.5
                # Exercise MoveIt's current-state monitor, as RViz does.
                motion.start_state.is_diff = True
                constraints = Constraints()
                constraints.joint_constraints = [
                    JointConstraint(joint_name=name, position=value,
                                    tolerance_above=0.0001, tolerance_below=0.0001, weight=1.0)
                    for name, value in target.items()]
                motion.goal_constraints = [constraints]
                response = wait(planner.call_async(request)).motion_plan_response
                assert response.error_code.val == MoveItErrorCodes.SUCCESS, (
                    f'{group} planning failed: {response.error_code.val}')
                points = response.trajectory.joint_trajectory.points
                assert len(points) >= 2, 'Empty trajectory'
                times = [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in points]
                assert times[-1] > 0 and all(b > a for a, b in zip(times, times[1:])), (
                    'Trajectory has invalid timing')
                goal = ExecuteTrajectory.Goal()
                goal.trajectory = response.trajectory
                handle = wait(executor.send_goal_async(goal))
                assert handle.accepted, 'Execution rejected'
                result = wait(handle.get_result_async(), timeout=30).result
                assert result.error_code.val == MoveItErrorCodes.SUCCESS, (
                    f'Execution failed: {result.error_code.val}')
                deadline = time.monotonic() + 3
                while time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.1)
                    actual = dict(zip(latest['msg'].name, latest['msg'].position))
                    if all(abs(actual[name] - value) < 0.001 for name, value in target.items()):
                        break
                else:
                    raise AssertionError(f'Feedback did not reach goal: {actual}')
                print(f'PASS: {group} plan and execute {target} ({len(points)} points)', flush=True)
            print(f'PASS: simulation validation complete. Log: {log_path}', flush=True)
        finally:
            os.killpg(launch.pid, signal.SIGINT)
            try:
                launch.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(launch.pid, signal.SIGKILL)
                launch.wait()
            node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
