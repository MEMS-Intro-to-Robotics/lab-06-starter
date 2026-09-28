"""The arm, the gripper, and the motion helpers for Lab 6.

This file is complete. Copy it into your package with the other starter files;
you do not need to edit it:

    cp scaffolds/arm.py ros2_ws/src/lab06_moveit/lab06_moveit/scripts/

pick_and_place.py uses the ArmNode class below. Its methods wrap the Lab 5
plan, check for None, execute, and wait sequence. Two methods add behavior:

  - close_gripper() stops waiting after a few seconds and returns the measured
    finger position. When the fingers close on a block they stop short of the
    commanded 0.8 and the gripper action never reports success, so waiting for
    it would wait forever.
  - move_to_pose() takes a speed factor, so the last few centimeters of a
    descent can be slower than the rest of the motion.
"""

from __future__ import annotations

import os
import signal
import sys
import threading
import time
from collections.abc import Callable

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionFK
from pymoveit2 import GripperCommand, MoveIt2
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState

from lab06_moveit.scripts.table import MOUNT_OFFSET

ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
GRIPPER_JOINT = "right_finger_bottom_joint"

# Joint configurations, radians, joint_1 first.
# RETRACT is the Gen3 Lite's own folded pose, the one the real arm parks in.
# From RETRACT most straight-line moves are impossible, so every run goes to
# WORK first: an elbow-up pose above the table from which the pick and place
# poses below are reachable.
RETRACT = [-0.0527, 0.3669, 2.59, -1.5359, -0.6988, -1.5189]
WORK = [0.40, 0.02, 2.27, -1.57, -0.84, 1.97]

# Gripper pointing straight down, as a quaternion (x, y, z, w).
TOP_DOWN = (0.0, 1.0, 0.0, 0.0)

# Heights of end_effector_link in base_link, meters. The arm's base sits
# MOUNT_OFFSET (0.0627) above the tabletop (see table.py), and the fingertips
# are GRIPPER_LENGTH below end_effector_link, so at GRASP_HEIGHT the fingertips
# are level with the middle of a 50 mm block resting on the table. These are
# the heights the real arm uses to stack blocks at the lab stations, measured
# on the arm: the open fingertips touch the table at end_effector_link z = 0.094.
GRIPPER_LENGTH = 0.094 + MOUNT_OFFSET  # 0.157
APPROACH_HEIGHT = 0.274
GRASP_HEIGHT = GRIPPER_LENGTH + 0.05 / 2.0 - MOUNT_OFFSET  # 0.119

# Finger positions (right_finger_bottom_joint). 0.0 is fully open (about 111 mm
# between the fingertips) and 0.8 is a firm close with nothing in the way.
GRIPPER_OPEN = 0.0
GRIPPER_CLOSED = 0.8

# Links allowed to touch a block the gripper is holding. Passed to
# attach_collision_object so MoveIt does not count the fingers squeezing the
# block as a collision.
TOUCH_LINKS = [
    "end_effector_link",
    "gripper_base_link",
    "left_finger_prox_link",
    "left_finger_dist_link",
    "right_finger_prox_link",
    "right_finger_dist_link",
]


class ArmNode(Node):
    """A ROS 2 node with MoveIt, the gripper, and measured joint positions."""

    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=ARM_JOINTS,
            base_link_name="base_link",
            end_effector_name="end_effector_link",
            group_name="arm",
        )
        # GripperCommand talks to the gripper's action server directly.
        # ignore_new_calls_while_executing=False lets open() replace a close
        # that is still pressing on a block; with True, that open is dropped.
        self.gripper = GripperCommand(
            node=self,
            gripper_joint_names=[GRIPPER_JOINT],
            open_gripper_joint_positions=[GRIPPER_OPEN],
            closed_gripper_joint_positions=[GRIPPER_CLOSED],
            ignore_new_calls_while_executing=False,
            gripper_command_action_name="/gen3_lite_2f_gripper_controller/gripper_cmd",
        )
        self._joint_positions: dict[str, float] = {}
        self._joint_stamp = 0.0
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self._fk_client = self.create_client(GetPositionFK, "compute_fk")

    # ------------------------------------------------------------ measured state

    def _on_joint_state(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            self._joint_positions[name] = position
        # The robot's own clock: simulated time in Gazebo, real time on the arm.
        self._joint_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def finger_position(self) -> float | None:
        """The latest measured gripper finger position, or None before the
        first /joint_states message arrives."""
        return self._joint_positions.get(GRIPPER_JOINT)

    def wait_until_ready(self, timeout: float = 30.0) -> bool:
        """Wait for MoveIt to answer before sending it anything."""
        if not self._fk_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(
                f"MoveIt did not start within {timeout:.0f} s. Is the sim.launch.py "
                "MoveIt launch still running in its own terminal?"
            )
            return False
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.moveit2.compute_fk() is not None:
                self.get_logger().info("MoveIt is ready.")
                return True
            time.sleep(1.0)
        self.get_logger().error(
            "MoveIt answered but could not compute the arm's current pose. Check "
            "the MoveIt terminal for errors."
        )
        return False

    # ------------------------------------------------------------------- motion

    def move_to_joints(self, joint_positions: list[float], attempts: int = 3) -> bool:
        """Plan and execute a joint-space motion, retrying a failed plan."""
        for attempt in range(1, attempts + 1):
            trajectory = self.moveit2.plan(joint_positions=joint_positions)
            if trajectory is None:
                self.get_logger().warn(f"Planning attempt {attempt} of {attempts} failed.")
                continue
            self.moveit2.execute(trajectory)
            if self.moveit2.wait_until_executed():
                return True
            self.get_logger().warn(f"Execution attempt {attempt} of {attempts} failed.")
        self.get_logger().error("Could not plan or execute this joint-space motion.")
        return False

    def move_to_pose(
        self,
        x: float,
        y: float,
        z: float,
        cartesian: bool = False,
        speed: float = 1.0,
        attempts: int = 3,
    ) -> bool:
        """Move end_effector_link to (x, y, z) in base_link, gripper pointing down.

        cartesian=True moves in a straight line and fails rather than bend the
        path. speed scales velocity and acceleration, 0.01 to 1.0.
        """
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = x, y, z
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ) = TOP_DOWN
        self.moveit2.max_velocity = speed
        self.moveit2.max_acceleration = speed
        try:
            for attempt in range(1, attempts + 1):
                trajectory = self.moveit2.plan(
                    pose=pose,
                    cartesian=cartesian,
                    max_step=0.005,
                    cartesian_fraction_threshold=0.9 if cartesian else 0.0,
                )
                if trajectory is None:
                    self.get_logger().warn(
                        f"Planning attempt {attempt} of {attempts} to "
                        f"({x:.3f}, {y:.3f}, {z:.3f}) failed."
                    )
                    continue
                self.moveit2.execute(trajectory)
                if self.moveit2.wait_until_executed():
                    return True
                self.get_logger().warn(f"Execution attempt {attempt} of {attempts} failed.")
        finally:
            self.moveit2.max_velocity = 0.0
            self.moveit2.max_acceleration = 0.0
        kind = "straight-line" if cartesian else "pose-goal"
        self.get_logger().error(
            f"Could not plan a {kind} motion to ({x:.3f}, {y:.3f}, {z:.3f})."
        )
        return False

    # ------------------------------------------------------------------ gripper

    def _move_fingers(self, position: float, timeout: float) -> float | None:
        """Command the fingers, wait until they stop moving or the timeout
        passes, and return the measured position.

        Times are read from /joint_states, not the wall clock, so a slow
        simulation gets the same amount of robot time as the real arm.
        """
        self.gripper.move_to_position(position)
        start = self._joint_stamp
        last = self.finger_position()
        still_since = start
        wall_limit = time.time() + 10.0 * timeout  # in case /joint_states stops
        while time.time() < wall_limit:
            time.sleep(0.05)
            now, stamp = self.finger_position(), self._joint_stamp
            if stamp - start > timeout:
                break
            if last is None or now is None or abs(now - last) > 0.002:
                last, still_since = now, stamp
            elif stamp - still_since > 0.5 and stamp - start > 0.5:
                break
        return self.finger_position()

    def open_gripper(self, position: float = GRIPPER_OPEN) -> float | None:
        """Open the fingers (fully, by default) and return the measured position."""
        return self._move_fingers(position, timeout=3.0)

    def close_gripper(self) -> float | None:
        """Close the fingers and return the measured position.

        With nothing between the fingers this reads about 0.80. With a block
        between them the fingers stop early and it reads less.
        """
        return self._move_fingers(GRIPPER_CLOSED, timeout=4.0)


def run(
    node: ArmNode,
    milestones: dict[int, Callable[[], object]],
    setup: Callable[[], object],
    default: list[int],
) -> None:
    """Spin the node, then run the milestones named on the command line, or
    the `default` ones when none are named."""
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    own_args = sys.argv[1:]
    if "--ros-args" in own_args:
        own_args = own_args[: own_args.index("--ros-args")]
    requested: list[int] = []
    for argument in own_args:
        if argument.isdigit() and int(argument) in milestones:
            requested.append(int(argument))
        else:
            valid = ", ".join(str(number) for number in sorted(milestones))
            node.get_logger().error(
                f"'{argument}' is not a milestone. Pass any of {valid}, or nothing to run all."
            )
            node.destroy_node()
            rclpy.try_shutdown()
            sys.exit(2)
    if not requested:
        requested = default

    try:
        if node.wait_until_ready():
            names = node.get_node_names()
            if "gz_ros_control" in names and "grasp_watcher" not in names:
                node.get_logger().warn(
                    "Gazebo is running but the grasp watcher is not, so blocks will slip "
                    "and the tower will fall. Start the simulation with the lab's launch "
                    "file, from the root of your Lab 6 repository: ros2 launch lab06_sim.launch.py"
                )
            setup()
            for number in requested:
                milestones[number]()
                time.sleep(1.0)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def init_ros() -> None:
    """Start ROS with a Ctrl+C handler that exits at once.

    rclpy's own handler tears the context down under a motion that is still
    running, which leaves the node waiting for a result that can never arrive.
    """
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    def stop(_signum: int, _frame: object) -> None:
        print("Interrupted. Stopping; the arm holds its position.", flush=True)
        os._exit(130)

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
