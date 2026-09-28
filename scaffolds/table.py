"""The table and the arm's mount in MoveIt's planning scene.

This file is complete. Copy it into your package with the other starter
files; you do not need to edit it or run it yourself:

    cp scaffolds/table.py ros2_ws/src/lab06_moveit/lab06_moveit/scripts/

At the lab stations the arm sits on a Kinova quick mount (50 mm tall), bolted
to a 12 x 12 x 0.5 inch plate on the table. lab06_sim.launch.py builds the same
station in Gazebo. add_table() adds matching collision boxes to MoveIt's
planning scene so the planner accounts for the table and mount. Gazebo and
MoveIt use separate models of the world (see the manual's Section 1.2).
"""

from __future__ import annotations

from pymoveit2 import MoveIt2
from rclpy.node import Node

GAZEBO_WORLD = "empty"
ROBOT_BASE_HEIGHT = 0.30  # Gazebo places base_link 0.30 m above its ground plane

# base_link is the center of the bottom of the arm's base, which rests on the
# quick mount. Heights below are in base_link, meters.
QUICK_MOUNT_SIZE = (0.11, 0.11, 0.050)
PLATE_SIZE = (0.3048, 0.3048, 0.0127)  # 12 x 12 x 0.5 in
MOUNT_OFFSET = QUICK_MOUNT_SIZE[2] + PLATE_SIZE[2]  # 0.0627: base_link above the tabletop
TABLE_TOP_Z = -MOUNT_OFFSET
TABLE_SIZE = (1.829, 1.016, 0.038)  # the lab benches, 6 ft x 40 in, mount in the middle

# A millimeter of air under the arm's base, so the base is not reported as
# touching the quick mount.
_GAP = 0.001


def add_table_to_planning_scene(moveit2: MoveIt2) -> None:
    """Add the table, plate, and quick mount as collision boxes."""
    boxes = {
        "table": (TABLE_SIZE, TABLE_TOP_Z - TABLE_SIZE[2] / 2.0),
        "mount_plate": (PLATE_SIZE, TABLE_TOP_Z + PLATE_SIZE[2] / 2.0),
        "quick_mount": (
            (QUICK_MOUNT_SIZE[0], QUICK_MOUNT_SIZE[1], QUICK_MOUNT_SIZE[2] - _GAP),
            TABLE_TOP_Z + PLATE_SIZE[2] + (QUICK_MOUNT_SIZE[2] - _GAP) / 2.0,
        ),
    }
    for object_id, (size, center_z) in boxes.items():
        moveit2.add_collision_box(
            id=object_id,
            size=size,
            position=(0.0, 0.0, center_z),
            quat_xyzw=(0.0, 0.0, 0.0, 1.0),
            frame_id="base_link",
        )


def add_table(node: Node, moveit2: MoveIt2) -> None:
    """Put the table and mount in MoveIt's planning scene."""
    add_table_to_planning_scene(moveit2)
    node.get_logger().info("Added the table and mount to MoveIt's planning scene.")
