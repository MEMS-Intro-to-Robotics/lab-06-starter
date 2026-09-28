"""Simulation only: hold a gripped block to the gripper in Gazebo.

This file is complete. Copy it into your package with the other starter files;
you do not need to edit or run it: lab06_sim.launch.py starts it.

Simulated finger contact is not reliable enough to stack blocks: a block slips
in the fingers and is pushed sideways when they open. This node holds the block
in Gazebo. It watches the gripper's finger joint, and
when the fingers stop partway closed with a block between them, it joins that
block to the gripper in Gazebo with a DetachableJoint. When the fingers open
again, it lets the block go.

It changes Gazebo only. Your code still has to attach and detach the block in
MoveIt's planning scene, exactly as it would on the real arm, where this node
is not run.
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.entity_plugin_v_pb2 import EntityPlugin_V
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.transport13 import Node as GzNode
from rclpy.node import Node
from sensor_msgs.msg import JointState

from lab06_moveit.scripts.arm import GRIPPER_JOINT
from lab06_moveit.scripts.blocks import (
    BLOCKS,
    GAZEBO_WORLD,
    HOLD_STATE_TOPIC,
    HOLD_TOPIC,
    ROBOT_BASE_HEIGHT,
)

ROBOT_MODEL = "gen3_lite"
HOLD_ATTACH_TOPIC = "/lab06/hold/{model}/attach"
GRIPPER_LINK = "end_effector_link"

# A gripped block's center sits this far below end_effector_link (the
# fingertips grip it at mid-height).
BLOCK_BELOW_GRIPPER = 0.15
# Fingers that stop between these positions have something between them:
# well below 0.80 on a block, 0.80 on nothing, 0.0 fully open.
HOLDING_ABOVE = 0.10
HOLDING_BELOW = 0.75
# How far the fingers must open from where they stopped before the block is
# let go, and how still they must be to count as stopped.
RELEASE_OPENING = 0.08
STILL = 0.002  # joint change per check, radians
# Seconds of robot time (the /joint_states clock). Shorter than the 0.5 s that
# arm.py waits, so the hold is in place before pick() lifts the block.
STILL_FOR = 0.3
NEAR = 0.03  # meters between the block and the point between the fingertips
# Gazebo answers slowly when the simulation runs below real time, so wait for
# its own report that a hold took effect rather than for the request's reply.
CONFIRM_WAIT = 10.0  # seconds


class GraspWatcher(Node):
    def __init__(self) -> None:
        super().__init__("grasp_watcher")
        self._gz = GzNode()
        self._lock = threading.Lock()
        self._poses: dict[str, tuple[int, float, float, float]] = {}
        self._gz.subscribe(Pose_V, f"/world/{GAZEBO_WORLD}/pose/info", self._on_poses)
        self._release_pubs = {
            model: self._gz.advertise(HOLD_TOPIC.format(model=model), Empty)
            for model, _, _ in BLOCKS.values()
        }
        self._attach_pubs = {
            model: self._gz.advertise(HOLD_ATTACH_TOPIC.format(model=model), Empty)
            for model, _, _ in BLOCKS.values()
        }
        # What Gazebo says each block's hold is doing: True once attached.
        self._attached: dict[str, bool] = {model: False for model, _, _ in BLOCKS.values()}
        for model in self._attached:
            self._gz.subscribe(
                StringMsg,
                HOLD_STATE_TOPIC.format(model=model),
                lambda msg, model=model: self._on_hold_state(model, msg),
            )

        self._finger: float | None = None
        self._stamp = 0.0
        self._last: float | None = None
        self._still_since = 0.0
        self._rest: float | None = None  # where the fingers last came to rest
        self._at_rest = False
        self._held: str | None = None  # Gazebo model name

        self._add_holds()
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self.create_timer(0.1, self._check)
        self.get_logger().info("Grasp watcher ready: holding gripped blocks in Gazebo.")

    def _add_holds(self) -> None:
        """Give each block one DetachableJoint to the gripper, added once.
        Gazebo attaches a new one at once, so let each go straight away; a
        grasp attaches it again on its attach topic."""
        while True:
            with self._lock:
                robot = self._poses.get(ROBOT_MODEL)
                present = all(model in self._poses for model in self._attached)
            if robot is not None and present:
                break
            self.get_logger().info(
                "Waiting for the arm and the three blocks in Gazebo "
                "(blocks reset adds missing blocks).",
                throttle_duration_sec=30.0,
            )
            time.sleep(0.5)
        for model in self._attached:
            request = EntityPlugin_V()
            request.entity.id = robot[0]
            plugin = request.plugins.add()
            plugin.name = "gz::sim::systems::DetachableJoint"
            plugin.filename = "gz-sim-detachable-joint-system"
            plugin.innerxml = (
                f"<parent_link>{GRIPPER_LINK}</parent_link><child_model>{model}</child_model>"
                f"<child_link>link</child_link><detach_topic>{HOLD_TOPIC.format(model=model)}</detach_topic>"
                f"<attach_topic>{HOLD_ATTACH_TOPIC.format(model=model)}</attach_topic>"
                f"<output_topic>{HOLD_STATE_TOPIC.format(model=model)}</output_topic>"
            )
            self._gz.request(
                f"/world/{GAZEBO_WORLD}/entity/system/add", request, EntityPlugin_V, Boolean,
                int(CONFIRM_WAIT * 1000),
            )
            self._wait_attached(model, True)
            self._release(model)
            if not self._wait_attached(model, False):
                self.get_logger().warn(f"Gazebo did not confirm releasing {model} at startup.")

    # ------------------------------------------------------------- inputs

    def _on_poses(self, msg: Pose_V) -> None:
        poses = {p.name: (p.id, p.position.x, p.position.y, p.position.z) for p in msg.pose}
        with self._lock:
            self._poses = poses

    def _on_hold_state(self, model: str, msg: StringMsg) -> None:
        self._attached[model] = msg.data.strip().lower() == "attached"

    def _wait_attached(self, model: str, attached: bool) -> bool:
        deadline = time.time() + CONFIRM_WAIT
        while time.time() < deadline:
            if self._attached[model] == attached:
                return True
            time.sleep(0.05)
        return False

    def _on_joint_state(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            if name == GRIPPER_JOINT:
                self._finger = position
                self._stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    # ------------------------------------------------------------- decisions

    def _check(self) -> None:
        """Act once each time the fingers come to rest: hold a block if they
        came to rest by closing on one, release it if they came to rest by
        opening. Resting after opening never grabs the block just let go."""
        finger, now = self._finger, self._stamp
        if finger is None:
            return
        if self._last is None or abs(finger - self._last) > STILL:
            self._still_since = now
            self._at_rest = False
        self._last = finger
        if self._at_rest or now - self._still_since < STILL_FOR:
            return
        self._at_rest = True
        previous, self._rest = self._rest, finger
        if previous is None:
            return
        moved = finger - previous  # positive = closed further

        if self._held is not None and moved < -RELEASE_OPENING:
            model = self._held
            self._release(model)
            self._held = None
            if self._wait_attached(model, False):
                self.get_logger().info(f"Released {model} (fingers at {finger:.3f}).")
            else:
                self.get_logger().warn(f"Gazebo did not confirm releasing {model}.")
        elif self._held is None and moved > RELEASE_OPENING and HOLDING_ABOVE < finger < HOLDING_BELOW:
            self._hold(finger)

    def _hold(self, finger: float) -> None:
        # A hold Gazebo applied after its request timed out is still a hold.
        stale = [model for model, attached in self._attached.items() if attached]
        if stale:
            self._held = stale[0]
            self.get_logger().info(f"Holding {stale[0]} (already attached in Gazebo).")
            return
        with self._lock:
            poses = dict(self._poses)
        gripper = poses.get(GRIPPER_LINK)  # in the robot model's frame = base_link
        if gripper is None:
            self.get_logger().warn("No gripper pose from Gazebo; cannot hold a block.")
            return
        target = (gripper[1], gripper[2], gripper[3] - BLOCK_BELOW_GRIPPER)
        nearest, distance = None, NEAR
        for model, _, _ in BLOCKS.values():
            pose = poses.get(model)  # world frame
            if pose is None:
                continue
            d = math.dist((pose[1], pose[2], pose[3] - ROBOT_BASE_HEIGHT), target)
            if d < distance:
                nearest, distance = model, d
        if nearest is None:
            self.get_logger().info(
                f"Fingers stopped at {finger:.3f} with no block between them; nothing to hold."
            )
            return
        self._attach_pubs[nearest].publish(Empty())
        if self._wait_attached(nearest, True):
            self._held = nearest
            self.get_logger().info(f"Holding {nearest} (fingers at {finger:.3f}).")
        else:
            self.get_logger().warn(f"Gazebo did not confirm holding {nearest}.")

    def _release(self, model: str) -> None:
        for _ in range(3):
            self._release_pubs[model].publish(Empty())
            time.sleep(0.1)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GraspWatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._held is not None:
            node._release(node._held)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
