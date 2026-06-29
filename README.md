# meshflow

**Onshape → ROS 2 simulation package, fully automated.**

meshflow takes an Onshape CAD assembly URL and produces a complete, simulation-ready ROS 2 description package — URDF, xacro, Gazebo plugins, launch files — with zero manual configuration. Robot structure is derived entirely from URDF geometry, so it works on any differential-drive robot regardless of how joints and links are named.

---

## What you get

Given one Onshape URL and a robot name, meshflow generates a fully structured ROS 2 package:

```
<robot>_description/
├── launch/
│   ├── display.launch.py      # RViz preview — no colcon needed
│   └── gazebo.launch.py       # full Gazebo Classic 11 simulation
├── models/
│   ├── urdf/
│   │   ├── <robot>.urdf       # flat URDF (used by display.launch)
│   │   └── <robot>.urdf.xacro # xacro with Gazebo plugins wired in
│   └── meshes/                # STL files pulled from Onshape
├── gazebo/
│   └── <robot>.gazebo         # diff-drive + lidar + friction plugins
└── rviz/
    └── robot.rviz             # pre-configured RViz layout
```

Plugins auto-generated based on detected geometry:
- **Differential drive** (`/cmd_vel` in, `/odom` out) — wheel separation and diameter measured from mesh
- **Lidar / ray sensor** (`/scan`, 360° @ 20 Hz, 12 m range) — for any elevated revolute joint
- **Joint state publisher** — keeps TF live for all moving joints
- **Friction and material** — per-link Gazebo surface properties

---

## Requirements

| Dependency | Version | Notes |
|---|---|---|
| Python | 3.10+ | |
| [uv](https://docs.astral.sh/uv/) | any | package manager — auto-installed by `install.sh` |
| ROS 2 | Humble or later | must be sourced before launching |
| Gazebo Classic | 11 | `gazebo_ros` bridge |
| `ros-$ROS_DISTRO-gazebo-ros-pkgs` | — | Gazebo ↔ ROS bridge |
| `ros-$ROS_DISTRO-robot-state-publisher` | — | TF broadcast |
| `ros-$ROS_DISTRO-joint-state-publisher-gui` | — | manual joint control in RViz |
| `ros-$ROS_DISTRO-xacro` | — | xacro processing |

Install the ROS 2 packages (replace `humble` with your distro):

```bash
sudo apt install \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro
```

---

## Installation

```bash
git clone https://github.com/prtmxio/meshflow
cd meshflow
bash install.sh
```

The script:
1. Checks Python 3.10+
2. Installs `uv` if not present
3. Installs all Python dependencies (`onshape-to-robot`, `numpy`, `trimesh`, `python-dotenv`)
4. Installs the `meshflow` CLI globally via `uv tool install` → available from anywhere as `meshflow`

If `meshflow` is not found after install, add `~/.local/bin` to your PATH:

```bash
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.zshrc   # or ~/.bashrc
source ~/.zshrc
```

---

## Setup — Onshape API keys

meshflow reads your CAD via the Onshape REST API. You need a free API key pair.

**Get your keys:**
1. Log in to [Onshape](https://cad.onshape.com)
2. Click your account avatar → **API Keys**
3. Click **Create new API key** → copy the Access Key and Secret Key

**Save them:**

```bash
meshflow init
```

This creates `~/.config/meshflow/.env` and opens it in your `$EDITOR`. Fill in:

```
ONSHAPE_ACCESS_KEY=<your access key>
ONSHAPE_SECRET_KEY=<your secret key>
```

Save and close. Run `meshflow init` again at any time to edit the keys.

---

## Usage

```bash
meshflow
```

You will be prompted for:

| Prompt | Example |
|---|---|
| Onshape URL | `https://cad.onshape.com/documents/abc.../w/def.../e/ghi...` |
| Robot name | `my_robot` |
| Assembly name | `asm` (the Onshape assembly tab name) |
| Output format | `urdf` (also: `sdf`, `mujoco`) |

The generated package lands in `output/<robot>_description/`.

---

## Run in Gazebo

**Step 1 — copy the package into your ROS 2 workspace:**

```bash
cp -r output/<robot>_description ~/ros2_ws/src/
```

**Step 2 — build:**

```bash
cd ~/ros2_ws
colcon build --packages-select <robot>_description
source install/setup.zsh   # or setup.bash
```

**Step 3 — launch:**

```bash
ros2 launch <robot>_description gazebo.launch.py
```

Gazebo, RViz, and the robot spawn automatically. To drive the robot:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

To verify the lidar:

```bash
ros2 topic hz /scan        # expect ~20 Hz
```

---

## Quick preview (no colcon)

To inspect the URDF in RViz without building a workspace:

```bash
cd output/<robot>_description/launch
ros2 launch display.launch.py
```

---

## Reference

```
meshflow --help       show usage summary
meshflow init         create/edit API key config
meshflow              run the converter
```

---

## Architecture

| Module | Role |
|---|---|
| `cli.py` | Entry point — prompts, subcommand dispatch |
| `onshape.py` | API auth, URL parsing, `onshape-to-robot` subprocess |
| `restructure.py` | File layout, `package://` URI patching, boilerplate |
| `detector.py` | `KinematicDAG` + `URDFTraits` — geometry-only classification |
| `generator.py` | `.gazebo` and xacro generation from `URDFTraits` |
| `templates.py` | All string templates as named constants |

### How geometry detection works

`KinematicDAG` parses the URDF `<joint>` tree and computes global 4×4 transforms for every link. `URDFTraits` then classifies each node using **only spatial and kinematic properties** — no name matching anywhere:

| Classification | Rule |
|---|---|
| Drive wheel | `continuous` joint, axis ≈ Y-axis, z_min < 20 cm |
| Passive contact (caster) | `fixed` joint, z_min < 1 cm |
| Revolute sensor | `revolute` joint, axis not Y, z_min > 5 cm |
| Fixed sensor | `fixed` leaf node, z_min > 5 cm, no co-located drive wheel sibling |

Wheel diameter and separation are measured directly from mesh geometry via `trimesh`, so plugin values are physically accurate without any manual input.

---

## Tests

```bash
uv run pytest tests/ -v
```
