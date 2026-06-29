import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

from .detector import KinematicDAG, URDFTraits
from .generator import generate_gazebo_file, generate_xacro, validate_xacro
from .onshape import CONFIG_FILE, build_config, load_api_keys, parse_onshape_url, run_conversion
from .restructure import restructure_for_ros2, validate_urdf


def _banner(text: str) -> None:
    width = 60
    print("\n" + "=" * width)
    print(f"  {text}")
    print("=" * width)


def ask(prompt: str, default: str = "") -> str:
    display = f"{prompt} [{default}]: " if default else f"{prompt}: "
    answer = input(display).strip()
    return answer if answer else default


def die(message: str) -> None:
    print(f"\n[ERROR] {message}", file=sys.stderr)
    sys.exit(1)


def _cmd_init() -> None:
    CONFIG_FILE.parent.mkdir(parents=True, exist_ok=True)

    if not CONFIG_FILE.exists():
        CONFIG_FILE.write_text(
            "ONSHAPE_ACCESS_KEY=your_access_key_here\n"
            "ONSHAPE_SECRET_KEY=your_secret_key_here\n"
        )
        print(f"\n  Created config at {CONFIG_FILE}")
        print("  Replace the placeholder values with your Onshape API keys.")
        print("  Get them at: Onshape → Account menu → API Keys\n")
    else:
        print(f"\n  Config exists at {CONFIG_FILE} — opening for editing.\n")

    editor = os.environ.get("EDITOR", "nano")
    try:
        subprocess.run([editor, str(CONFIG_FILE)])
    except FileNotFoundError:
        print(f"  Could not open editor '{editor}'. Edit the file manually:")
        print(f"    {CONFIG_FILE}\n")
        return

    print(f"\n  Done. Run 'meshflow' from anywhere to start converting.\n")


def _cmd_help() -> None:
    print("""
meshflow — Onshape → ROS 2 package generator

USAGE
  meshflow [command]

COMMANDS
  (none)    Convert an Onshape assembly URL → full ROS 2 description package
  init      Create or edit the API key config (~/.config/meshflow/.env)
  --help    Show this help message

WORKFLOW
  1. meshflow init         Set your Onshape API keys (one-time setup)
  2. meshflow              Paste an Onshape URL and answer prompts
  3. cp -r output/<pkg> ~/ros2_ws/src/
     cd ~/ros2_ws && colcon build --packages-select <pkg>
     source install/setup.zsh
     ros2 launch <pkg> gazebo.launch.py

OUTPUT (urdf mode)
  <pkg>/models/urdf/<robot>.urdf          flat URDF  (display.launch.py)
  <pkg>/models/urdf/<robot>.urdf.xacro    xacro with Gazebo plugins included
  <pkg>/gazebo/<robot>.gazebo             diff-drive + lidar + friction plugins
  <pkg>/launch/display.launch.py          RViz preview (no colcon needed)
  <pkg>/launch/gazebo.launch.py           full Gazebo simulation

DOCS
  https://github.com/prtmxio/meshflow
""")


def main() -> None:
    if len(sys.argv) > 1 and sys.argv[1] in ("--help", "-h", "help"):
        _cmd_help()
        return

    if len(sys.argv) > 1 and sys.argv[1] == "init":
        _cmd_init()
        return

    access_key, secret_key = load_api_keys()
    os.environ["ONSHAPE_API"]        = "https://cad.onshape.com"
    os.environ["ONSHAPE_ACCESS_KEY"] = access_key
    os.environ["ONSHAPE_SECRET_KEY"] = secret_key

    _banner("Onshape → URDF Converter (ROS 2 Edition)")

    url = ask("\nOnshape URL")
    if not url:
        die("No URL entered.")
    ids = parse_onshape_url(url)

    print()
    robot_name    = ask("Robot name",    default="my_robot")
    assembly_name = ask("Assembly name", default="asm")

    output_format = ask("Output format (urdf/sdf/mujoco)", default="urdf")
    if output_format not in ("urdf", "sdf", "mujoco"):
        die(f"Invalid output format '{output_format}'.")

    macro_based = False
    if output_format == "urdf":
        macro_based = ask("Macro-based xacro? (y/N)", default="n").lower() == "y"
        if macro_based:
            print("  [WARN] Macro-based xacro is DDR-specific and requires standard link names.")
            print("         Use flat xacro (default) for the universal generalised pipeline.")

    safe_name  = re.sub(r"[^\w\-]", "_", robot_name).lower()
    script_dir = Path(__file__).parent.parent  # meshflow repo root

    if output_format == "urdf":
        pkg_name    = f"{safe_name}_description"
        output_dir  = script_dir / "output" / pkg_name
        staging_dir = output_dir / ".staging"

        if output_dir.exists():
            shutil.rmtree(output_dir)
        staging_dir.mkdir(parents=True, exist_ok=True)

        config = build_config(ids, safe_name, assembly_name, output_format)
        (staging_dir / "config.json").write_text(json.dumps(config, indent=4))

        _banner("Extracting Kinematics")
        run_conversion(staging_dir)

        restructure_for_ros2(staging_dir, output_dir, safe_name, pkg_name)
        validate_urdf(output_dir, safe_name)
        generate_xacro(output_dir, safe_name, macro_based=macro_based)
        validate_xacro(output_dir, safe_name)

        # Detect traits and generate Gazebo plugins
        urdf_path = output_dir / "models" / "urdf" / f"{safe_name}.urdf"
        if urdf_path.exists():
            mesh_dir = output_dir / "models" / "meshes"
            dag    = KinematicDAG(urdf_path, mesh_dir=mesh_dir)
            traits = URDFTraits.from_dag(dag)
            generate_gazebo_file(output_dir, safe_name, pkg_name, traits)
        else:
            print("  [WARN] URDF not found — skipping Gazebo plugin generation.")

        _banner("Done!")
        print(f"""
  Option A – launch directly (no colcon needed):
    cd {output_dir.resolve()}/launch
    ros2 launch display.launch.py

  Option B – build as a proper ROS 2 package (resolves package:// URIs natively):
    cp -r {output_dir.resolve()} ~/ros2_ws/src/
    cd ~/ros2_ws && colcon build --packages-select {pkg_name}
    source install/setup.zsh
    ros2 launch {pkg_name} display.launch.py

  Files in models/urdf/:
    {safe_name}.urdf        ← flat URDF (used by display.launch.py)
    {safe_name}.urdf.xacro  ← xacro for Gazebo simulation

  Gazebo simulation:
    ros2 launch {pkg_name} gazebo.launch.py

  RViz tip: Fixed Frame = odom when running Gazebo.
""")

    else:
        output_dir = script_dir / "output" / safe_name
        if output_dir.exists():
            shutil.rmtree(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)

        config = build_config(ids, safe_name, assembly_name, output_format)
        (output_dir / "config.json").write_text(json.dumps(config, indent=4))

        _banner(f"Extracting {output_format.upper()} Kinematics")
        run_conversion(output_dir)

        _banner("Done!")
        print(f"\n  Raw {output_format.upper()} files generated in:\n    {output_dir.resolve()}\n")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
