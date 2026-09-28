"""Lab 6 simulation: Gazebo with the Gen3 Lite, the lab station, and the grasp watcher.

Run it from the root of your Lab 6 repository, in a prepared container
terminal (your workspace sourced), and leave it running:

    ros2 launch lab06_sim.launch.py

The launch starts these components in order:
  1. Gazebo with the Kinova Gen3 Lite and its controllers (the same launch as
     Lab 5, with the lab's arguments filled in);
  2. the lab station: the table, and the arm's mounting plate and quick mount,
     modeled from the real parts at the lab stations, and the three blocks;
  3. the grasp watcher (grasp_watcher.py), which holds a gripped block in
     Gazebo because simulated finger contact alone cannot stack blocks.

MoveIt and RViz are started separately, in their own terminal, as in Lab 5.

It reads the station and block layout from your package (blocks.py), so run
it after building, in a terminal where your workspace is sourced. Run from
anywhere else, ros2 launch reports that it cannot find lab06_sim.launch.py.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from lab06_moveit.scripts.blocks import BLOCKS, STATION_MODEL, block_sdf, spawn_z, station_sdf

MODELS = Path(__file__).resolve().parent / "models"


def generate_launch_description() -> LaunchDescription:
    kinova = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("kortex_bringup"), "/launch/kortex_sim_control.launch.py"]
        ),
        launch_arguments={
            "sim_gazebo": "true",
            "robot_type": "gen3_lite",
            "gripper": "gen3_lite_2f",
            "robot_name": "gen3_lite",
            "dof": "6",
            "launch_rviz": "false",
            "use_sim_time": "true",
            "robot_controller": "joint_trajectory_controller",
        }.items(),
    )
    station = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-string", station_sdf(MODELS), "-name", STATION_MODEL, "-allow_renaming", "false"],
        output="screen",
    )
    cube = f"file://{(MODELS / 'companion_cube.stl').as_posix()}"
    blocks = [
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-string", block_sdf(model, rgb, cube), "-name", model, "-allow_renaming", "false",
                "-x", str(x), "-y", str(y), "-z", str(spawn_z()),
            ],
            output="screen",
        )
        for model, (x, y), rgb in BLOCKS.values()
    ]
    watcher = Node(package="lab06_moveit", executable="grasp_watcher", output="screen")
    return LaunchDescription([
        kinova,
        # Gazebo needs a few seconds before it accepts new models, and the
        # watcher needs the finger joint's state. If a spawn fails anyway,
        # `blocks reset` (which every milestone runs) adds what is missing.
        TimerAction(period=5.0, actions=[station]),
        TimerAction(period=8.0, actions=blocks),
        TimerAction(period=15.0, actions=[watcher]),
    ])
