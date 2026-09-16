#!/usr/bin/python3
"""Send one planned gripper move through MoveIt's standard MoveGroup action."""

import sys

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class MoveItGripperSmoke(Node):
    """Request and observe one planned parallel-gripper movement."""

    def __init__(self):
        """Create the MoveGroup action client and user parameters."""
        super().__init__('moveit_gripper_smoke')
        self.declare_parameter('target', 0.01)
        self.declare_parameter('joint', 'finger_stroke')
        self.declare_parameter('group', 'gripper')
        self._client = ActionClient(self, MoveGroup, 'move_action')

    def run(self):
        """Execute one plan and return whether MoveIt reported success."""
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                'MoveGroup action was not available within 10 seconds')
            return False

        target = self.get_parameter('target').value
        goal = MoveGroup.Goal()
        goal.request.group_name = self.get_parameter('group').value
        goal.request.num_planning_attempts = 3
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 1.0
        goal.request.max_acceleration_scaling_factor = 1.0
        constraint = Constraints()
        constraint.joint_constraints.append(JointConstraint(
            joint_name=self.get_parameter('joint').value,
            position=target,
            tolerance_above=1e-4,
            tolerance_below=1e-4,
            weight=1.0,
        ))
        goal.request.goal_constraints.append(constraint)
        goal.planning_options.plan_only = False
        goal.planning_options.replan = False

        goal_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=10.0)
        handle = goal_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('MoveGroup rejected the gripper goal')
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=20.0)
        wrapped_result = result_future.result()
        if wrapped_result is None:
            self.get_logger().error(
                'MoveGroup did not finish within 20 seconds')
            return False
        code = wrapped_result.result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f'MoveGroup failed with error code {code}')
            return False

        self.get_logger().info(
            f'MoveIt executed {self.get_parameter("joint").value} target {target:.6f}')
        return True


def main():
    """Run the installed MoveIt execution check."""
    rclpy.init()
    node = MoveItGripperSmoke()
    try:
        success = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
