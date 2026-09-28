# Simulation models

`lab06_sim.launch.py` uses these meshes and textures to draw the lab station in
Gazebo. The physics uses simple boxes of the same size.

| File | What it is | Source |
|---|---|---|
| `mount_plate.stl` | 12 x 12 x 0.5 in aluminum mounting plate | course CAD (Evan Kusa), converted from STEP |
| `quick_mount.stl` | Kinova quick mount, 110 x 110 x 50 mm | course CAD (Evan Kusa), converted from STEP |
| `pine_table.png` | butcher-block pine texture for the lab bench | generated for this lab |
| `companion_cube.stl` | the blocks' shape, 50 mm | see below |

`companion_cube.stl` is "Companion Cube" by petruvius
(https://www.printables.com/model/280778-companion-cube), licensed under
CC BY 4.0 (http://creativecommons.org/licenses/by/4.0/). Changes: recentered on
its middle and scaled from 68.4 mm to 50 mm.
