#!/usr/bin/env python3
"""Run an isolated mock-arm obstacle avoidance demonstration."""
import argparse
from copy import deepcopy
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import (
    CollisionObject, Constraints, DisplayTrajectory, JointConstraint,
    MoveItErrorCodes, PlanningScene, RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, GetPositionFK, GetStateValidity
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive


class Demo:
    def __init__(self, node):
        self.node = node
        self.latest = None
        self.received = 0.0
        self.subscription = node.create_subscription(JointState, '/joint_states', self.update, 10)
        self.validity = self.client(GetStateValidity, '/check_state_validity')
        self.planner = self.client(GetMotionPlan, '/plan_kinematic_path')
        self.fk = self.client(GetPositionFK, '/compute_fk')
        self.scene = self.client(ApplyPlanningScene, '/apply_planning_scene')
        self.executor = ActionClient(node, ExecuteTrajectory, '/execute_trajectory')
        if not self.executor.wait_for_server(timeout_sec=40):
            raise RuntimeError('Execution action unavailable')
        self.display = node.create_publisher(DisplayTrajectory, '/display_planned_path', 10)
        deadline = time.monotonic() + 40
        while self.latest is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if self.latest is None:
            raise RuntimeError('No joint feedback')

    def update(self, msg):
        if set(f'joint_{i}' for i in range(1, 7)) | {'jaw1', 'jaw2'} <= set(msg.name):
            self.latest = msg
            self.received = time.monotonic()

    def wait(self, future, timeout=45):
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError('ROS request timed out')
        return future.result()

    def client(self, kind, name):
        client = self.node.create_client(kind, name)
        if not client.wait_for_service(timeout_sec=45):
            raise RuntimeError(f'Service unavailable: {name}')
        return client

    def state(self, values=None, names=None):
        state = RobotState()
        state.joint_state = deepcopy(self.latest)
        state.joint_state.header.stamp.sec = 0
        state.joint_state.header.stamp.nanosec = 0
        if values is not None:
            updates = dict(zip(names or [f'joint_{i}' for i in range(1, 7)], values))
            state.joint_state.position = [updates.get(n, p) for n, p in zip(
                state.joint_state.name, state.joint_state.position)]
        return state

    def valid(self, state):
        request = GetStateValidity.Request()
        request.robot_state = state
        # Empty group checks the complete robot, including the gripper.
        return self.wait(self.validity.call_async(request)).valid

    def plan(self, start, goal):
        request = GetMotionPlan.Request()
        motion = request.motion_plan_request
        motion.group_name = 'arm'
        motion.pipeline_id = 'ompl'
        # Use this configuration's default OMPL planner (RRTConnect).
        motion.start_state = start
        motion.allowed_planning_time = 10.0
        motion.num_planning_attempts = 5
        motion.max_velocity_scaling_factor = 0.5
        motion.max_acceleration_scaling_factor = 0.5
        constraints = Constraints()
        constraints.joint_constraints = [
            JointConstraint(joint_name=f'joint_{i}', position=value,
                            tolerance_above=0.001, tolerance_below=0.001, weight=1.0)
            for i, value in enumerate(goal, 1)]
        motion.goal_constraints = [constraints]
        result = self.wait(self.planner.call_async(request)).motion_plan_response
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f'Planning failed: {result.error_code.val}')
        if len(result.trajectory.joint_trajectory.points) < 2:
            raise RuntimeError('Planner returned an empty trajectory')
        return result.trajectory

    def samples(self, trajectory):
        joints = trajectory.joint_trajectory
        yield self.state(joints.points[0].positions, joints.joint_names)
        for a, b in zip(joints.points, joints.points[1:]):
            steps = max(1, math.ceil(max(abs(y - x) for x, y in zip(
                a.positions, b.positions)) / 0.01))
            for step in range(1, steps + 1):
                yield self.state([x + (y - x) * step / steps for x, y in zip(
                    a.positions, b.positions)], joints.joint_names)

    def box(self, position=None, size=0.08):
        obj = CollisionObject()
        obj.header.frame_id = 'world'
        obj.id = 'obstacle_demo_box'
        obj.operation = CollisionObject.REMOVE if position is None else CollisionObject.ADD
        if position is not None:
            pose = Pose()
            pose.position = position
            pose.orientation.w = 1.0
            obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[size] * 3)]
            obj.primitive_poses = [pose]
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.world.collision_objects = [obj]
        request = ApplyPlanningScene.Request(scene=scene)
        if not self.wait(self.scene.call_async(request)).success:
            raise RuntimeError('Failed to apply planning scene')

    def place_obstacle(self, baseline, start, goal):
        points = baseline.joint_trajectory.points
        for fraction in (0.5, 0.4, 0.6):
            point = points[round((len(points) - 1) * fraction)]
            middle = self.state(point.positions, baseline.joint_trajectory.joint_names)
            request = GetPositionFK.Request()
            request.header.frame_id = 'world'
            request.fk_link_names = ['Link_6_1', 'Link_5_1', 'Link_4_1']
            request.robot_state = middle
            result = self.wait(self.fk.call_async(request))
            if result.error_code.val != MoveItErrorCodes.SUCCESS:
                raise RuntimeError('Forward kinematics failed')
            for pose in result.pose_stamped:
                for size in (0.08, 0.06, 0.04):
                    self.box(pose.pose.position, size)
                    if self.valid(start) and self.valid(goal) and not self.valid(middle):
                        p = pose.pose.position
                        print(f'PASS: {size:.2f} m box at world ({p.x:.3f}, {p.y:.3f}, '
                              f'{p.z:.3f}) blocks baseline; endpoints remain clear', flush=True)
                        return
        self.box()
        raise RuntimeError('Could not place a blocking box with collision-free endpoints')

    def execute(self, trajectory):
        goal = ExecuteTrajectory.Goal(trajectory=trajectory)
        handle = self.wait(self.executor.send_goal_async(goal))
        if not handle.accepted:
            raise RuntimeError('Execution rejected')
        end = trajectory.joint_trajectory.points[-1]
        duration = end.time_from_start.sec + end.time_from_start.nanosec * 1e-9
        result = self.wait(handle.get_result_async(), timeout=duration + 20).result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f'Execution failed: {result.error_code.val}')
        target = dict(zip(trajectory.joint_trajectory.joint_names, end.positions))
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            actual = dict(zip(self.latest.name, self.latest.position))
            if time.monotonic() - self.received < 1 and all(
                    abs(actual[n] - p) < 0.002 for n, p in target.items()):
                return
        raise RuntimeError('Joint feedback did not reach the commanded goal')

    def run(self):
        start_values = [-0.65, 0.6, 0.0, 0.0, 0.0, 0.0]
        goal_values = [0.65, 0.6, 0.0, 0.0, 0.0, 0.0]
        start, goal = self.state(start_values), self.state(goal_values)
        if not self.valid(start) or not self.valid(goal):
            raise RuntimeError('Demo start or goal is in collision')
        self.execute(self.plan(self.state(), start_values))
        print('PASS: reached demo start configuration', flush=True)
        start = self.state()
        baseline = self.plan(start, goal_values)
        if not all(self.valid(state) for state in self.samples(baseline)):
            raise RuntimeError('Unobstructed baseline failed collision validation')
        print('PASS: unobstructed baseline planned and validated', flush=True)
        self.place_obstacle(baseline, start, goal)
        trajectory = self.plan(start, goal_values)
        count = 0
        for state in self.samples(trajectory):
            if not self.valid(state):
                raise RuntimeError('Detour failed dense collision validation')
            count += 1
        print(f'PASS: detour collision-free at {count} samples (<= 0.01 rad spacing)', flush=True)
        self.display.publish(DisplayTrajectory(trajectory_start=start, trajectory=[trajectory]))
        self.execute(trajectory)
        print('PASS: executed detour and verified final joint feedback', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless', action='store_true', help='Disable RViz and exit after validation')
    parser.add_argument('--domain-id', type=int, default=174, help='Unused isolated ROS domain (default: 174)')
    parser.add_argument('--log', default='/tmp/arctos-obstacle-demo.log')
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    os.environ.pop('ROS_LOCALHOST_ONLY', None)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    log_path = Path(args.log)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('arctos_obstacle_demo')
    try:
        with log_path.open('w') as log:
            launch = subprocess.Popen(
                ['ros2', 'launch', 'arctos_moveit_config', 'demo.launch.py',
                 f'use_rviz:={str(not args.headless).lower()}'],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                Demo(node).run()
                print(f'Demo complete. Launch log: {log_path}', flush=True)
                if not args.headless:
                    input('Press Enter to close the simulation... ')
            finally:
                os.killpg(launch.pid, signal.SIGINT)
                try:
                    launch.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    os.killpg(launch.pid, signal.SIGKILL)
                    launch.wait()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
