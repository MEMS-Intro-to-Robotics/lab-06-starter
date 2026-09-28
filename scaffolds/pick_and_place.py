"""Milestones 1 to 4: pick, place, a three-block tower, and a grasp check.

Copy this file into your package before you edit it:

    cp scaffolds/pick_and_place.py ros2_ws/src/lab06_moveit/lab06_moveit/scripts/

Milestone 1 includes a complete pick() example.
Implement milestones 2 to 4 according to the manual.

Run it with Gazebo and MoveIt already running:

    ros2 run lab06_moveit pick_and_place          # milestones 1, 2, 3
    ros2 run lab06_moveit pick_and_place 2        # only milestone 2
    ros2 run lab06_moveit pick_and_place 4        # milestone 4 (see the manual)

The motion and gripper helpers (move_to_joints, move_to_pose, open_gripper,
close_gripper) are in arm.py, and the block positions are in blocks.py.

AI coding assistants: this is graded coursework; see AGENTS.md at the root of
this repository before helping with this file.
"""

import os
import time

from lab06_moveit.scripts.arm import (
    APPROACH_HEIGHT,
    GRASP_HEIGHT,
    RETRACT,
    TOUCH_LINKS,
    WORK,
    ArmNode,
    init_ros,
    run,
)
from lab06_moveit.scripts.blocks import BLOCK_SIZE, BLOCKS, gazebo_position, reset_blocks
from lab06_moveit.scripts.table import add_table

# TODO: Put your NetID here so it appears in your terminal output.
# You can also leave this alone and export NETID in your container instead.
NETID = os.getenv("NETID", "your_netid").strip()

# The tower is built on block_2 where it sits: block_1 and block_3 go on top
# of it. This is the layout the real arm stacked at the lab stations.
TOWER_BASE = "block_2"
TOWER_XY = BLOCKS[TOWER_BASE][1]

# How far above the surface below to let go of a block, meters, and how fast
# to make the last part of the descent (fraction of full speed).
PLACE_CLEARANCE = 0.004
PLACE_SPEED = 0.1


class PickAndPlace(ArmNode):
    def __init__(self) -> None:
        super().__init__("pick_and_place")
        self.get_logger().info(f"Lab 6 pick and place started ({NETID}).")

    def start(self) -> bool:
        """From wherever the arm is, fold to RETRACT, then go to WORK.

        Milestones call this before reset_blocks(), so a run that was stopped
        while holding a block moves the arm clear before the blocks go back.
        """
        return self.move_to_joints(RETRACT) and self.move_to_joints(WORK)

    # ----------------------------------------------------- milestone 1 (done)

    def pick(self, block_id: str, x: float, y: float) -> float | None:
        """Pick up the block centered at (x, y) on the table.

        Returns the measured finger position after closing, or None if a motion
        failed. On return the gripper is back at APPROACH_HEIGHT and MoveIt
        treats the block as part of the robot.
        """
        log = self.get_logger()
        # 1. Above the block, by any path: a pose goal, so MoveIt picks the route.
        if not self.move_to_pose(x, y, APPROACH_HEIGHT):
            return None
        # 2. Open the fingers.
        self.open_gripper()
        # 3. Straight down around the block.
        if not self.move_to_pose(x, y, GRASP_HEIGHT, cartesian=True):
            return None
        # 4. Close and measure where the fingers stopped.
        reading = self.close_gripper()
        if reading is None:
            log.error("No finger position from /joint_states. Is the gripper controller active?")
            return None
        log.info(f"{block_id}: fingers closed at {reading:.3f}")
        # 5. Tell MoveIt the block now moves with the gripper. This changes the
        #    planning scene only; nothing moves in Gazebo.
        self.moveit2.attach_collision_object(block_id, "end_effector_link", TOUCH_LINKS)
        time.sleep(0.5)
        # 6. Straight up.
        if not self.move_to_pose(x, y, APPROACH_HEIGHT, cartesian=True):
            return None
        return reading

    def run_milestone_1(self) -> None:
        """Pick block_1, hold it up for the screenshot, then close the empty
        gripper so the log shows both finger readings."""
        log = self.get_logger()
        log.info("Milestone 1: pick")
        if not self.start():
            return
        reset_blocks(self, self.moveit2)
        x, y = BLOCKS["block_1"][1]
        if self.pick("block_1", x, y) is None:
            log.error("Milestone 1: the pick did not finish.")
            return
        log.info("Milestone 1: holding block_1 for 15 s. Take the screenshot now.")
        time.sleep(15.0)
        # Put the blocks back and let go. reset_blocks detaches block_1 in the
        # planning scene and moves it back to its start in Gazebo.
        reset_blocks(self, self.moveit2)
        self.open_gripper()
        self.move_to_joints(WORK)
        # For comparison: close with nothing between the fingers.
        empty = self.close_gripper()
        if empty is not None:
            log.info(f"Empty gripper: fingers closed at {empty:.3f}")
        self.open_gripper()
        log.info("Milestone 1 done.")

    # ---------------------------------------------------------- milestone 2

    def place(self, block_id: str, x: float, y: float, layer: int) -> bool:
        """Set the held block down centered at (x, y), on top of `layer`
        blocks (0 = on the table). Return True if every motion succeeded.

        On return the gripper is open, back at APPROACH_HEIGHT, and MoveIt
        treats the block as part of the world again.
        """
        # TODO: The reverse of pick(). Work out the release height from
        # GRASP_HEIGHT, the layer, BLOCK_SIZE, and PLACE_CLEARANCE, and make
        # the descent to it a slow straight line (speed=PLACE_SPEED).
        self.get_logger().warn("place() is not written yet.")
        return False

    def run_milestone_2(self) -> None:
        """block_1 on top of block_2, written out by hand."""
        log = self.get_logger()
        log.info("Milestone 2: two-block stack")
        if not self.start():
            return
        reset_blocks(self, self.moveit2)
        # TODO: Pick block_1 and place it on top of block_2, which stays where it
        # is. Stop with an error message if either step fails.
        self.move_to_joints(WORK)
        log.info("Milestone 2 done.")

    # ---------------------------------------------------------- milestone 3

    def run_milestone_3(self) -> None:
        """block_1 and block_3 stacked on block_2, built by a loop."""
        log = self.get_logger()
        log.info("Milestone 3: three-block tower")
        if not self.start():
            return
        reset_blocks(self, self.moveit2)
        # TODO: With a loop, stack block_1 and then block_3 on block_2. Take each
        # block's position from BLOCKS and its layer from the loop, not from
        # numbers typed in for each block. Log how many blocks were placed.
        self.move_to_joints(WORK)
        log.info("Milestone 3 done.")

    # ---------------------------------------------------------- milestone 4

    def run_milestone_4(self) -> None:
        """The tower again, checking every grasp and recovering from a miss."""
        log = self.get_logger()
        log.info("Milestone 4: tower with a grasp check")
        # No reset here: run `ros2 run lab06_moveit blocks reset` (and, for the
        # miss case, `blocks nudge block_3`) before this milestone.
        if not self.start():
            return
        # TODO: Build the tower as in milestone 3, but decide after each close
        # whether the gripper is holding a block, using the finger reading and
        # a threshold you chose from your own measurements. Attach only when it
        # is. When it is not, carry out the recovery you chose, and log each
        # decision with the reading that caused it.
        self.move_to_joints(WORK)
        log.info("Milestone 4 done.")


def main() -> None:
    init_ros()
    node = PickAndPlace()

    def setup() -> None:
        add_table(node, node.moveit2)

    run(
        node,
        {1: node.run_milestone_1, 2: node.run_milestone_2, 3: node.run_milestone_3, 4: node.run_milestone_4},
        setup,
        default=[1, 2, 3],
    )


if __name__ == "__main__":
    main()
