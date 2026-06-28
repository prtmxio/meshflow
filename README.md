# meshflow

Onshape → ROS 2 package generator for DDR robots.

Converts an Onshape CAD assembly URL into a complete, simulation-ready ROS 2 package (URDF, xacro, Gazebo plugins, launch files) without requiring knowledge of joint names, link names, or robot-specific configuration. Everything is derived from URDF geometry.

## Install

```bash
cd meshflow/
uv pip install -e .
```

## Usage

```bash
meshflow
# → paste Onshape URL, answer 3 questions
# → get a complete ROS 2 package in output/<robot_name>_description/
```

## Run in Gazebo

```bash
cp -r output/<robot_name>_description ~/ros2_ws/src/
cd ~/ros2_ws
colcon build --packages-select <robot_name>_description
source install/setup.zsh
# Uncomment the xacro:include line in the .urdf.xacro file, then:
ros2 launch <robot_name>_description gazebo.launch.py
```

## Requirements

- Python 3.10+
- `uv` with `onshape-to-robot` installed
- ROS 2 (Humble or later) for `check_urdf`, `xacro`, and launch
- Gazebo Classic 11 for simulation
- `trimesh` (optional but recommended — used for accurate wheel radius from mesh geometry)

## Architecture

| Module | Role |
|---|---|
| `cli.py` | Interactive entry point — same prompts as convert.py |
| `onshape.py` | API auth, URL parsing, onshape-to-robot subprocess |
| `restructure.py` | File layout, URI patching, boilerplate generation |
| `detector.py` | `KinematicDAG` + `URDFTraits` — geometry-only classification |
| `generator.py` | `.gazebo` and xacro generation using `URDFTraits` |
| `templates.py` | All string templates as named constants |

## How detection works

`KinematicDAG` builds a directed tree from `<joint>` parent/child relationships, computing global 4×4 transforms. `URDFTraits` classifies every node using **only** spatial/geometric properties — no name matching:

| Classification | Rule |
|---|---|
| Drive wheel | `continuous` joint + axis ≈ Y-axis + z_min < 20 cm |
| Passive contact | `fixed` joint + z_min < 1 cm (touches ground) |
| Revolute sensor | `revolute` joint + elevated (z_min > 5 cm) |
| Fixed sensor | `fixed` leaf node + elevated |

Works on any DDR robot regardless of joint/link naming convention, language, or geometry asymmetry.

## Tests

```bash
uv run pytest tests/ -v
# Drop a URDF at tests/test_robot.urdf to run the full integration tests
```
