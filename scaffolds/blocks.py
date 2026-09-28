"""The three blocks, in Gazebo and in MoveIt's planning scene.

This file is complete. Copy it into your package with the other starter files;
you do not need to edit it:

    cp scaffolds/blocks.py ros2_ws/src/lab06_moveit/lab06_moveit/scripts/

Gazebo simulates block positions. MoveIt plans from a separate planning scene
that your code updates. This file initializes both at the start of a run and
lets you compare their positions afterwards.

It is also a command you can run in its own terminal:

    ros2 run lab06_moveit blocks reset          # blocks back to their start
    ros2 run lab06_moveit blocks where          # Gazebo vs. planning scene
    ros2 run lab06_moveit blocks nudge block_2  # move block_2 in Gazebo only
"""

from __future__ import annotations

import sys
import threading
import time
from pathlib import Path

import rclpy
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.entity_factory_pb2 import EntityFactory
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from moveit_msgs.msg import ObjectColor, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from pymoveit2 import MoveIt2
from rclpy.executors import MultiThreadedExecutor
from rclpy.logging import get_logger
from rclpy.node import Node
from std_msgs.msg import ColorRGBA

from lab06_moveit.scripts.table import (
    GAZEBO_WORLD,
    PLATE_SIZE,
    QUICK_MOUNT_SIZE,
    ROBOT_BASE_HEIGHT,
    TABLE_SIZE,
    TABLE_TOP_Z,
    add_table,
    add_table_to_planning_scene,
)

ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

BLOCK_SIZE = 0.05  # meters, a cube (the 3D-printed lab blocks)

# Planning-scene id -> (Gazebo model name, (x, y) of the block's center in
# base_link, RGB color). The layout used at the lab stations: a row at
# x = 0.44, 0.18 m apart.
BLOCKS: dict[str, tuple[str, tuple[float, float], str]] = {
    "block_1": ("lab06_block_red", (0.44, -0.168), "0.85 0.10 0.10"),
    "block_2": ("lab06_block_blue", (0.44, 0.012), "0.10 0.10 0.85"),
    "block_3": ("lab06_block_yellow", (0.44, 0.192), "0.90 0.85 0.10"),
}

STATION_MODEL = "lab06_station"  # the table and mount in Gazebo

# Where `blocks nudge` moves a block: off to the side, still in reach, where
# the planning scene does not expect it.
NUDGE_XY = (0.32, 0.10)

# Mass, friction, and contact settings for the simulated blocks.
BLOCK_MASS = 0.03  # kg
_INERTIA = BLOCK_MASS * BLOCK_SIZE**2 / 6.0


def block_sdf(model: str, rgb: str, mesh_uri: str | None = None) -> str:
    """One block for Gazebo: a 50 mm box to the physics engine, drawn as the
    mesh at mesh_uri (lab06_sim.launch.py passes the Companion Cube) or as a
    plain box. Single-quoted attributes, so it can sit inside a quoted request."""
    s = BLOCK_SIZE
    look = (
        f"<mesh><uri>{mesh_uri}</uri></mesh>" if mesh_uri
        else f"<box><size>{s} {s} {s}</size></box>"
    )
    return (
        f"<?xml version='1.0'?><sdf version='1.8'><model name='{model}'><link name='link'>"
        f"<inertial><mass>{BLOCK_MASS}</mass><inertia><ixx>{_INERTIA}</ixx><iyy>{_INERTIA}</iyy>"
        f"<izz>{_INERTIA}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>"
        f"<collision name='c'><geometry><box><size>{s} {s} {s}</size></box></geometry>"
        "<surface><friction><ode><mu>2.0</mu><mu2>2.0</mu2></ode>"
        "<torsional><coefficient>0.5</coefficient><use_patch_radius>1</use_patch_radius>"
        "<patch_radius>0.02</patch_radius></torsional></friction>"
        "<contact><ode><kp>1e5</kp><kd>10.0</kd><max_vel>0.10</max_vel>"
        "<min_depth>1e-5</min_depth></ode></contact></surface></collision>"
        f"<visual name='v'><geometry>{look}</geometry>"
        f"<material><ambient>{rgb} 1</ambient><diffuse>{rgb} 1</diffuse></material></visual>"
        "</link></model></sdf>"
    )


def station_sdf(models: Path | None = None) -> str:
    """The lab station for Gazebo, as one static model: table, plate, and quick
    mount. With `models` (the starter's models/ folder) the plate and quick
    mount are drawn from their CAD meshes and the table in pine; without it
    they are plain boxes. Either way they collide as plain boxes, which costs
    the physics engine far less."""
    tx, ty, tz = TABLE_SIZE
    px, py, pz = PLATE_SIZE
    qx, qy, qz = QUICK_MOUNT_SIZE
    top = ROBOT_BASE_HEIGHT + TABLE_TOP_Z  # 0.2373 in Gazebo
    table_z, plate_z, quick_z = top - tz / 2, top + pz / 2, top + pz + qz / 2
    if models is None:
        pine = ""
        plate_look = f"<pose>0 0 {plate_z} 0 0 0</pose><geometry><box><size>{px} {py} {pz}</size></box></geometry>"
        quick_look = f"<pose>0 0 {quick_z} 0 0 0</pose><geometry><box><size>{qx} {qy} {qz}</size></box></geometry>"
    else:
        # Both meshes have their origin at the center of their bottom face.
        pine = f"<pbr><metal><albedo_map>file://{(models / 'pine_table.png').as_posix()}</albedo_map><metalness>0.0</metalness><roughness>0.7</roughness></metal></pbr>"
        plate_look = f"<pose>0 0 {top} 0 0 0</pose><geometry><mesh><uri>file://{(models / 'mount_plate.stl').as_posix()}</uri></mesh></geometry>"
        quick_look = f"<pose>0 0 {top + pz} 0 0 0</pose><geometry><mesh><uri>file://{(models / 'quick_mount.stl').as_posix()}</uri></mesh></geometry>"
    # One link: in a static model every link must be joined to something, and a
    # single link with several shapes is the simplest way to do that.
    return f"""<?xml version='1.0'?>
<sdf version='1.8'><model name='{STATION_MODEL}'><static>true</static>
<link name='station'>
  <collision name='table'><pose>0 0 {table_z} 0 0 0</pose>
    <geometry><box><size>{tx} {ty} {tz}</size></box></geometry></collision>
  <visual name='table'><pose>0 0 {table_z} 0 0 0</pose>
    <geometry><box><size>{tx} {ty} {tz}</size></box></geometry>
    <material><ambient>0.87 0.75 0.55 1</ambient><diffuse>{'1 1 1 1' if pine else '0.87 0.75 0.55 1'}</diffuse>{pine}</material></visual>
  <collision name='plate'><pose>0 0 {plate_z} 0 0 0</pose>
    <geometry><box><size>{px} {py} {pz}</size></box></geometry></collision>
  <visual name='plate'>{plate_look}
    <material><ambient>0.7 0.71 0.73 1</ambient><diffuse>0.8 0.81 0.83 1</diffuse><specular>0.9 0.9 0.9 1</specular><pbr><metal><metalness>0.25</metalness><roughness>0.45</roughness></metal></pbr></material></visual>
  <collision name='quick_mount'><pose>0 0 {quick_z} 0 0 0</pose>
    <geometry><box><size>{qx} {qy} {qz}</size></box></geometry></collision>
  <visual name='quick_mount'>{quick_look}
    <material><ambient>0.03 0.03 0.03 1</ambient><diffuse>0.05 0.05 0.05 1</diffuse><specular>0.2 0.2 0.2 1</specular><pbr><metal><metalness>0.0</metalness><roughness>0.6</roughness></metal></pbr></material></visual>
</link>
</model></sdf>"""


def spawn_z() -> float:
    """Gazebo world height to spawn a block resting on the table: a millimeter
    of drop, so it never starts inside the table."""
    return ROBOT_BASE_HEIGHT + center_z(0) + 0.001


def center_z(layer: int) -> float:
    """Height of a block's center in base_link when it sits on `layer` blocks
    (0 = on the table)."""
    return TABLE_TOP_Z + BLOCK_SIZE / 2.0 + layer * BLOCK_SIZE


# ------------------------------------------------------------------- Gazebo
#
# Gazebo is reached through its own transport library rather than the gz
# command, so each call costs milliseconds instead of starting a process.

HOLD_TOPIC = "/lab06/hold/{model}/detach"  # grasp_watcher releases a hold here
HOLD_STATE_TOPIC = "/lab06/hold/{model}/state"  # Gazebo reports "attached"/"detached"

_gz_node: GzNode | None = None
_gz_lock = threading.Lock()
_gz_poses: dict[str, tuple[int, float, float, float]] = {}
_gz_publishers: dict[str, object] = {}


def _on_poses(msg: Pose_V) -> None:
    poses = {p.name: (p.id, p.position.x, p.position.y, p.position.z) for p in msg.pose}
    with _gz_lock:
        _gz_poses.clear()
        _gz_poses.update(poses)


def _gz() -> GzNode:
    global _gz_node
    if _gz_node is None:
        _gz_node = GzNode()
        _gz_node.subscribe(Pose_V, f"/world/{GAZEBO_WORLD}/pose/info", _on_poses)
    return _gz_node


def gazebo_models(wait: float = 3.0) -> dict[str, tuple[int, float, float, float]]:
    """Every entity Gazebo reports: name -> (id, x, y, z). Top-level models
    are in the world frame. Empty if Gazebo is not running."""
    _gz()
    deadline = time.time() + wait
    while True:
        with _gz_lock:
            if _gz_poses:
                return dict(_gz_poses)
        if time.time() > deadline:
            return {}
        time.sleep(0.05)


def gazebo_position(block_id: str) -> tuple[float, float, float] | None:
    """Where Gazebo has the block's center, in base_link coordinates, or None
    if Gazebo does not answer."""
    pose = gazebo_models().get(BLOCKS[block_id][0])
    if pose is None:
        return None
    return (pose[1], pose[2], pose[3] - ROBOT_BASE_HEIGHT)


def _pose(x: float, y: float, z: float) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = x, y, z + ROBOT_BASE_HEIGHT
    pose.orientation.w = 1.0
    return pose


def _set_gazebo_position(model: str, x: float, y: float, z: float) -> bool:
    request = _pose(x, y, z)
    request.name = model
    ok, reply = _gz().request(f"/world/{GAZEBO_WORLD}/set_pose", request, Pose, Boolean, 3000)
    return bool(ok and reply.data)


def _create_block(model: str, x: float, y: float, z: float, rgb: str) -> bool:
    request = EntityFactory()
    request.sdf = block_sdf(model, rgb)
    request.name = model
    request.allow_renaming = False
    request.pose.CopyFrom(_pose(x, y, z))
    ok, reply = _gz().request(f"/world/{GAZEBO_WORLD}/create", request, EntityFactory, Boolean, 3000)
    return bool(ok and reply.data)


def _create_station() -> bool:
    request = EntityFactory()
    request.sdf = station_sdf()
    request.name = STATION_MODEL
    request.allow_renaming = False
    ok, reply = _gz().request(f"/world/{GAZEBO_WORLD}/create", request, EntityFactory, Boolean, 3000)
    return bool(ok and reply.data) or _arrives(STATION_MODEL, 0.0, 0.0)


def release_in_gazebo(block_id: str) -> None:
    """Release a block that grasp_watcher is holding to the gripper."""
    topic = HOLD_TOPIC.format(model=BLOCKS[block_id][0])
    if topic not in _gz_publishers:
        _gz_publishers[topic] = _gz().advertise(topic, Empty)
        time.sleep(0.3)  # let the watcher's joint discover the new publisher
    _gz_publishers[topic].publish(Empty())


def _reset_gazebo(node: Node, models: dict[str, tuple[int, float, float, float]]) -> None:
    for block_id in BLOCKS:
        release_in_gazebo(block_id)
    for block_id, (model, (x, y), rgb) in BLOCKS.items():
        # A millimeter of drop, so a block never starts inside the table.
        z = center_z(0) + 0.001
        if model in models:
            ok = _set_gazebo_position(model, x, y, z)
        else:
            # lab06_sim.launch.py spawns the blocks; a plain box stands in if it
            # did not.
            ok = _create_block(model, x, y, z, rgb)
        if not ok and not _arrives(model, x, y):
            node.get_logger().warn(f"Gazebo did not accept {block_id} ({model}).")


def _arrives(model: str, x: float, y: float, wait: float = 10.0) -> bool:
    """Whether Gazebo reports the model at (x, y) within `wait` seconds. A slow
    simulation can answer a request after its timeout and still carry it out."""
    deadline = time.time() + wait
    while time.time() < deadline:
        pose = gazebo_models().get(model)
        if pose is not None and abs(pose[1] - x) < 0.01 and abs(pose[2] - y) < 0.01:
            return True
        time.sleep(0.2)
    return False


# --------------------------------------------------------------- planning scene

_color_clients: dict[Node, object] = {}


def _reset_planning_scene(moveit2: MoveIt2) -> None:
    for block_id, (_model, (x, y), _rgb) in BLOCKS.items():
        moveit2.detach_collision_object(block_id)
        moveit2.add_collision_box(
            id=block_id,
            size=(BLOCK_SIZE, BLOCK_SIZE, BLOCK_SIZE),
            position=(x, y, center_z(0)),
            quat_xyzw=(0.0, 0.0, 0.0, 1.0),
            frame_id="base_link",
        )


def color_blocks(node: Node) -> None:
    """Draw each block in RViz in its Gazebo color. MoveIt draws every
    collision object in one color unless the planning scene gives it its own."""
    scene = PlanningScene(is_diff=True)
    for block_id, (_model, _xy, rgb) in BLOCKS.items():
        r, g, b = (float(c) for c in rgb.split())
        scene.object_colors.append(ObjectColor(id=block_id, color=ColorRGBA(r=r, g=g, b=b, a=1.0)))
    if node not in _color_clients:
        _color_clients[node] = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    client = _color_clients[node]
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().warn("MoveIt did not answer, so RViz shows the blocks in its default color.")
        return
    # A service rather than a message on /planning_scene: a message sent the
    # moment a publisher is created can be lost before move_group connects.
    future = client.call_async(ApplyPlanningScene.Request(scene=scene))
    deadline = time.time() + 5.0
    while not future.done() and time.time() < deadline:
        time.sleep(0.05)


def reset_blocks(node: Node, moveit2: MoveIt2) -> None:
    """Put all three blocks back at their start positions, in Gazebo and in
    the planning scene. Nothing is attached to the gripper afterwards.

    Adds the table first if it is missing, so the blocks have something to
    sit on."""
    models = gazebo_models()
    if models:
        if STATION_MODEL not in models:
            # The launch file spawns it; this covers a spawn that failed.
            if _create_station():
                node.get_logger().info("Added the lab station (table and mount) to Gazebo.")
            else:
                node.get_logger().warn(
                    "Gazebo has no lab station (table and mount) and did not accept one, so "
                    "the blocks will fall. Stop the simulation and start it again with: "
                    "ros2 launch lab06_sim.launch.py"
                )
        add_table(node, moveit2)
        _reset_gazebo(node, models)
    else:
        # On the real arm there is no Gazebo; only the planning scene changes.
        add_table_to_planning_scene(moveit2)
    _reset_planning_scene(moveit2)
    color_blocks(node)
    time.sleep(1.5)  # let the blocks settle and the planning scene update
    where = "in Gazebo and in the planning scene" if models else "in the planning scene (no Gazebo)"
    node.get_logger().info(f"Blocks reset {where}.")


def planning_scene_positions(moveit2: MoveIt2) -> dict[str, tuple[str, tuple[float, float, float] | None]]:
    """Where the planning scene has each block: ('world', center) or ('attached',
    None) when it is attached to the gripper."""
    positions: dict[str, tuple[str, tuple[float, float, float] | None]] = {}
    # The service client needs a moment to find MoveIt after the node starts.
    deadline = time.time() + 10.0
    while not moveit2.update_planning_scene():
        if time.time() > deadline:
            return positions
        time.sleep(0.5)
    if moveit2.planning_scene is None:
        return positions
    scene = moveit2.planning_scene
    for obj in scene.world.collision_objects:
        if obj.id in BLOCKS:
            p = obj.pose.position
            if obj.primitive_poses:
                q = obj.primitive_poses[0].position
                p = type(p)(x=p.x + q.x, y=p.y + q.y, z=p.z + q.z)
            positions[obj.id] = ("world", (p.x, p.y, p.z))
    for attached in scene.robot_state.attached_collision_objects:
        if attached.object.id in BLOCKS:
            positions[attached.object.id] = ("attached", None)
    return positions


# ---------------------------------------------------------------------- command

def _fmt(point: tuple[float, float, float] | None) -> str:
    if point is None:
        return "unknown"
    return f"({point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f})"


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    argv = sys.argv[1:]
    command = argv[: argv.index("--ros-args")] if "--ros-args" in argv else argv
    log = get_logger("lab06_blocks")

    if len(command) == 2 and command[0] == "nudge" and command[1] in BLOCKS:
        # Gazebo only; no MoveIt needed.
        if not gazebo_models():
            log.error("Gazebo is not running, so there is nothing to nudge.")
        elif _set_gazebo_position(BLOCKS[command[1]][0], NUDGE_XY[0], NUDGE_XY[1], center_z(0) + 0.001):
            log.info(
                f"Moved {command[1]} in Gazebo to ({NUDGE_XY[0]:.2f}, {NUDGE_XY[1]:.2f}). "
                "The planning scene still has it at its start position."
            )
        else:
            log.error("Gazebo did not move the block.")
        rclpy.try_shutdown()
        return
    if command not in (["reset"], ["where"]):
        log.error("Usage: ros2 run lab06_moveit blocks reset | where | nudge block_1|block_2|block_3")
        rclpy.try_shutdown()
        sys.exit(2)

    node = Node("lab06_blocks")
    moveit2 = MoveIt2(
        node=node,
        joint_names=ARM_JOINTS,
        base_link_name="base_link",
        end_effector_name="end_effector_link",
        group_name="arm",
    )
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        if command == ["reset"]:
            time.sleep(1.0)  # let MoveIt's planning scene topics connect
            reset_blocks(node, moveit2)
        else:
            positions = planning_scene_positions(moveit2)
            print(f"{'block':8} {'Gazebo':26} planning scene")
            for block_id in BLOCKS:
                state, point = positions.get(block_id, ("missing", None))
                entry = "attached to the gripper" if state == "attached" else (
                    _fmt(point) if state == "world" else "not in the scene")
                print(f"{block_id:8} {_fmt(gazebo_position(block_id)):26} {entry}")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
