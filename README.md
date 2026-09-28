# Lab 6: Pick-and-Place and Multi-Block Stacking: [Your Name]

ECE 383 / ME 555: Introduction to Robotics and Automation (Fall 2026)

Update this README with your name, NetID, and a 1–3 line summary of your work.

## Contents

- `scaffolds/`: the starting files for your package
  - `pick_and_place.py`: milestone 1 is complete; implement milestones 2 to 4
  - `arm.py`: complete; the arm, the gripper, and the motion helpers your
    code calls. Do not edit it.
  - `blocks.py`: complete; puts the three blocks in Gazebo and in MoveIt, and
    provides the `blocks reset`, `blocks where`, and `blocks nudge` commands.
    Do not edit it.
  - `grasp_watcher.py`: complete, simulation only; holds a gripped block to the
    gripper in Gazebo while the fingers are closed on it. Do not edit it.
  - `table.py`: complete; puts the table and mount in MoveIt's planning scene.
    Do not edit it.
- `lab06_sim.launch.py`: starts the simulation: Gazebo with the arm, the lab
  station (table, plate, and quick mount), and the grasp watcher
- `models/`: the station's plate, quick mount, and table texture, and the block mesh
- `check_package.py`: checks your package setup before the first build
- `lab06.rviz`: the RViz layout for this lab
- `docs/`: your four milestone screenshots
- `test_lab_6.py`: automated repository checks
- `pytest.ini`: limits `pytest` to `test_lab_6.py` so it skips the ROS 2 workspace

Create `ros2_ws/src/lab06_moveit/` by following the lab manual, then copy each
file from `scaffolds/` into the package's `scripts/` folder. Leave the originals
in `scaffolds/` as a clean copy.

## Run the grading checks

From the repository root on the VM:

```bash
pytest -v
```

Run it from the repository root, not from `ros2_ws/`. Started anywhere else,
`pytest` collects the test files that `ros2 pkg create` generated inside your
package and reports errors about `ament_copyright` and `ament_flake8` instead of
checking your submission.

The checks confirm that the repository contains your package, the five script
files, four readable images, an updated README, and entry points for all three
executables. They also flag tracked ROS 2 build output. Screenshot filenames and
the executable names in `setup.py` are recommendations, so reasonable
alternatives still pass.

The repository checks cover files and package setup. Course staff use your
Gradescope PDF to evaluate what your milestones do, the evidence in each
screenshot and log, and your discussion answers.

Before the final push, fix every failure and confirm that Classroom 50 reports a
passing result.
