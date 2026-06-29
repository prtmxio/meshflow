import re
import shutil
import subprocess
import tempfile
from pathlib import Path
from xml.etree import ElementTree as ET

from .templates import (
    CMAKE_TEMPLATE,
    DISPLAY_LAUNCH_TEMPLATE,
    GAZEBO_LAUNCH_TEMPLATE,
    GAZEBO_LAUNCH_NONWHEELED_TEMPLATE,
    PACKAGE_XML_TEMPLATE,
    RVIZ_TEMPLATE,
)


def _banner(text: str) -> None:
    width = 60
    print("\n" + "=" * width)
    print(f"  {text}")
    print("=" * width)


def restructure_for_ros2(staging_dir: Path, output_dir: Path, robot_name: str, pkg_name: str) -> None:
    _banner("Restructuring for ROS 2 Architecture")

    models_dir = output_dir / "models"
    urdf_dir   = models_dir / "urdf"
    meshes_dir = models_dir / "meshes"
    config_dir = output_dir / "config"
    rviz_dir   = output_dir / "rviz"
    launch_dir = output_dir / "launch"
    gazebo_dir = output_dir / "gazebo"

    for d in [urdf_dir, meshes_dir, config_dir, rviz_dir, launch_dir, gazebo_dir]:
        d.mkdir(parents=True, exist_ok=True)

    # 1. Isolate config
    config_src = staging_dir / "config.json"
    if config_src.exists():
        shutil.move(str(config_src), str(config_dir / "config.json"))

    # 2. Isolate URDF with canonical name
    for f in staging_dir.glob("*.urdf"):
        shutil.move(str(f), str(urdf_dir / f"{robot_name}.urdf"))
        break  # only one expected

    # 3. Flatten all STLs into meshes/ regardless of nesting depth
    assets_dir   = staging_dir / "assets"
    mesh_src_dir = assets_dir if assets_dir.exists() else staging_dir
    for f in mesh_src_dir.rglob("*.stl"):
        dest = meshes_dir / f.name
        if dest.exists():
            print(f"  [WARN] Duplicate mesh name, overwriting: {f.name}")
        shutil.move(str(f), str(dest))

    # 4. Patch URDF mesh URIs
    urdf_path = urdf_dir / f"{robot_name}.urdf"
    if urdf_path.exists():
        content = urdf_path.read_text()
        content = re.sub(
            r'package://[^"]+/([^"/]+\.stl)',
            rf'package://{pkg_name}/models/meshes/\1',
            content,
        )
        urdf_path.write_text(content)
        print(f"  Patched mesh URIs → package://{pkg_name}/models/meshes/<name>.stl")
        _patch_joint_link_collisions(urdf_path)

    # 5. Inherit saved RViz config if present next to the script, else generate default
    source_rviz = Path.cwd() / "robot.rviz"
    if source_rviz.exists():
        shutil.copy(str(source_rviz), str(rviz_dir / "robot.rviz"))
        print("  Inherited robot.rviz from working directory.")
    else:
        fixed_frame = _urdf_root_link(urdf_dir / f"{robot_name}.urdf")
        _write_default_rviz(rviz_dir / "robot.rviz", fixed_frame)
        print(f"  Generated default robot.rviz (RobotModel + TF + Grid, Fixed Frame = {fixed_frame}).")

    # 6. Generate ROS 2 package boilerplate
    _write_package_xml(output_dir, pkg_name)
    _write_cmake(output_dir, pkg_name)

    # 7. Generate launch files (gazebo.launch.py rewritten later with correct robot_kind)
    _write_launch_file(launch_dir, robot_name, pkg_name)
    write_gazebo_launch(launch_dir, robot_name, pkg_name, robot_kind="wheeled")

    # 8. Annihilate the sandbox
    shutil.rmtree(staging_dir, ignore_errors=True)
    print(f"  Package [{pkg_name}] structured successfully.")


def _write_package_xml(output_dir: Path, pkg_name: str) -> None:
    content = PACKAGE_XML_TEMPLATE.replace('PKG_NAME', pkg_name)
    (output_dir / "package.xml").write_text(content)
    print("  Generated package.xml")


def _write_cmake(output_dir: Path, pkg_name: str) -> None:
    content = CMAKE_TEMPLATE.replace('PKG_NAME', pkg_name)
    (output_dir / "CMakeLists.txt").write_text(content)
    print("  Generated CMakeLists.txt")


def _patch_joint_link_collisions(urdf_path: Path) -> None:
    content = urdf_path.read_text()
    root_el = ET.fromstring(content)
    link_names = {l.get('name') for l in root_el.findall('link')}
    collisions = [j.get('name') for j in root_el.findall('joint')
                  if j.get('name') in link_names]
    for name in collisions:
        print(f"  [WARN] Joint/link name collision: renaming joint '{name}' → '{name}_joint'")
        content = re.sub(
            r'(<joint\s[^>]*?\bname=")' + re.escape(name) + r'"',
            r'\g<1>' + name + r'_joint"',
            content,
            count=1,
        )
    if collisions:
        urdf_path.write_text(content)


def _urdf_root_link(urdf_path: Path) -> str:
    try:
        root_el = ET.parse(urdf_path).getroot()
        all_links   = {l.get('name') for l in root_el.findall('link')}
        child_links = set()
        for j in root_el.findall('joint'):
            c = j.find('child')
            if c is not None:
                child_links.add(c.get('link'))
        candidates = sorted(all_links - child_links)
        if candidates:
            return candidates[0]
    except Exception:
        pass
    return "base_link"


def _write_default_rviz(dest: Path, fixed_frame: str = "base_link") -> None:
    dest.write_text(RVIZ_TEMPLATE.replace("Fixed Frame: base_link", f"Fixed Frame: {fixed_frame}"))


def _write_launch_file(launch_dir: Path, robot_name: str, pkg_name: str) -> None:
    content = DISPLAY_LAUNCH_TEMPLATE.replace("ROBOT_NAME", robot_name)
    (launch_dir / "display.launch.py").write_text(content)
    print("  Generated launch/display.launch.py")


def write_gazebo_launch(launch_dir: Path, robot_name: str, pkg_name: str,
                        robot_kind: str = "wheeled") -> None:
    template = (GAZEBO_LAUNCH_TEMPLATE if robot_kind == "wheeled"
                else GAZEBO_LAUNCH_NONWHEELED_TEMPLATE)
    content = (
        template
        .replace('ROBOT_NAME', robot_name)
        .replace('PKG_NAME',   pkg_name)
    )
    (launch_dir / 'gazebo.launch.py').write_text(content)
    print(f"  Generated launch/gazebo.launch.py  [{robot_kind}]")


def validate_urdf(output_dir: Path, robot_name: str) -> None:
    urdf_path = output_dir / "models" / "urdf" / f"{robot_name}.urdf"
    checker   = shutil.which("check_urdf")
    if checker and urdf_path.exists():
        print(f"\n  Validating URDF with check_urdf …")
        result = subprocess.run([checker, str(urdf_path)], capture_output=True, text=True)
        if result.returncode == 0:
            print("  URDF validation passed.")
        else:
            print("  [WARN] URDF validation reported issues:\n", result.stdout or result.stderr)
