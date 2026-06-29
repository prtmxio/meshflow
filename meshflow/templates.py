"""
All multi-line string templates. No logic here — just named constants.
Substitution is done with .replace('ROBOT_NAME', x) etc. in the callers.
"""

DISPLAY_LAUNCH_TEMPLATE = """import re
import sys
from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_root  = Path(__file__).resolve().parent.parent
    urdf_path = pkg_root / "models" / "urdf" / "ROBOT_NAME.urdf"
    rviz_path = pkg_root / "rviz" / "robot.rviz"

    if not urdf_path.exists():
        print(f"[ERROR] URDF not found: {urdf_path}", file=sys.stderr)
        raise SystemExit(1)

    # Auto-discover every STL anywhere in the package tree, build name→path map.
    # This is resilient to the STLs ending up in meshes/, assets/, or the root.
    stl_map = {f.name: f for f in pkg_root.rglob("*.stl")}
    if not stl_map:
        print("[ERROR] No STL files found under package root!", file=sys.stderr)
        print(f"        Searched: {pkg_root}", file=sys.stderr)
        raise SystemExit(1)
    sample_dir = next(iter(stl_map.values())).parent
    print(f"[INFO] Found {len(stl_map)} STL file(s) in: {sample_dir}")

    robot_description = urdf_path.read_text()

    def _resolve(m: re.Match) -> str:
        name = m.group(1)
        path = stl_map.get(name)
        if path is None:
            print(f"[WARN] STL not found in package: {name}", file=sys.stderr)
            return m.group(0)   # leave original so RViz gives a clear error
        return f"file://{path}"

    robot_description = re.sub(
        r'package://[^"]+/([^"/]+\\.stl)',
        _resolve,
        robot_description,
    )

    patched = robot_description.count("file://")
    if patched == 0:
        print("[WARN] No mesh URIs were resolved — check URDF mesh filenames.", file=sys.stderr)
    else:
        print(f"[INFO] Resolved {patched} mesh URI(s) to absolute file:// paths.")

    rviz_args = (
        ["-d", str(rviz_path)]
        if rviz_path.exists() and rviz_path.stat().st_size > 0
        else []
    )

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": False,
            }],
            output="screen",
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=rviz_args,
            output="screen",
        ),
    ])
"""

GAZEBO_LAUNCH_TEMPLATE = """import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess, IncludeLaunchDescription,
    SetEnvironmentVariable, TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name   = 'PKG_NAME'
    pkg_path   = get_package_share_directory(pkg_name)
    gazebo_pkg = get_package_share_directory('gazebo_ros')

    # ── Xacro → URDF, pre-resolve package:// → file:// ───────────────────
    # Gazebo converts package:// → model:// during URDF→SDF which breaks
    # model DB lookup. Replacing with file:// means Gazebo loads STLs
    # directly from disk — no model database lookup needed.
    xacro_file = os.path.join(pkg_path, 'models', 'urdf', 'ROBOT_NAME.urdf.xacro')
    urdf_str   = xacro.process_file(xacro_file).toxml()
    urdf_str   = urdf_str.replace(
        f'package://{pkg_name}/',
        f'file://{pkg_path}/'
    )
    robot_description = {'robot_description': urdf_str}

    # ── Environment ────────────────────────────────────────────────────────
    # Disable blocking HTTP fetch to models.gazebosim.org on every startup.
    # Without this, gzserver hangs >30s and spawn_entity always times out.
    disable_online_db = SetEnvironmentVariable(
        name='GAZEBO_MODEL_DATABASE_URI', value=''
    )
    # Keep system Gazebo models (sun, ground_plane) findable.
    system_models  = '/usr/share/gazebo-11/models'
    existing       = os.environ.get('GAZEBO_MODEL_PATH', '')
    fix_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=system_models + (':' + existing if existing else '')
    )

    # ── gzserver (physics) — gui:=false, we start gzclient separately ─────
    # The default gazebo.launch.py injects libgazebo_ros_eol_gui.so which
    # causes shadow_caster timeouts and kills gzclient. Starting bare
    # gzclient ourselves avoids that entirely.
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'verbose': 'true',
            'gui':     'false',
        }.items(),
    )

    # ── gzclient (GUI) ─────────────────────────────────────────────────────
    gzclient = TimerAction(
        period=4.0,
        actions=[ExecuteProcess(cmd=['gzclient'], output='screen')]
    )

    # ── Robot State Publisher ──────────────────────────────────────────────
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': True}],
        output='screen',
    )

    # ── Spawn robot (delayed — gzserver needs ~3s with model DB disabled) ──
    spawn = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'ROBOT_NAME',
                    '-x', '0', '-y', '0', '-z', '0.05',
                ],
                output='screen',
            )
        ]
    )

    # ── RViz (use_sim_time=True so TF timestamps match Gazebo sim clock) ───
    rviz = TimerAction(
        period=13.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', os.path.join(pkg_path, 'rviz', 'robot.rviz')],
                parameters=[{'use_sim_time': True}],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        disable_online_db,   # must come before gazebo starts
        fix_model_path,
        gzserver,
        rsp,
        gzclient,
        spawn,
        rviz,
    ])
"""

GAZEBO_LAUNCH_NONWHEELED_TEMPLATE = """import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess, IncludeLaunchDescription,
    SetEnvironmentVariable, TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name   = 'PKG_NAME'
    pkg_path   = get_package_share_directory(pkg_name)
    gazebo_pkg = get_package_share_directory('gazebo_ros')

    xacro_file = os.path.join(pkg_path, 'models', 'urdf', 'ROBOT_NAME.urdf.xacro')
    urdf_str   = xacro.process_file(xacro_file).toxml()
    urdf_str   = urdf_str.replace(
        f'package://{pkg_name}/',
        f'file://{pkg_path}/'
    )
    robot_description = {'robot_description': urdf_str}

    disable_online_db = SetEnvironmentVariable(
        name='GAZEBO_MODEL_DATABASE_URI', value=''
    )
    system_models  = '/usr/share/gazebo-11/models'
    existing       = os.environ.get('GAZEBO_MODEL_PATH', '')
    fix_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=system_models + (':' + existing if existing else '')
    )

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'verbose': 'true', 'gui': 'false'}.items(),
    )

    gzclient = TimerAction(
        period=4.0,
        actions=[ExecuteProcess(cmd=['gzclient'], output='screen')]
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': True}],
        output='screen',
    )

    spawn = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'ROBOT_NAME',
                    '-x', '0', '-y', '0', '-z', '0.05',
                ],
                output='screen',
            )
        ]
    )

    # Load ros2_control controllers after spawn
    load_joint_state_broadcaster = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                output='screen',
            )
        ]
    )

    load_joint_trajectory_controller = TimerAction(
        period=17.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'joint_trajectory_controller',
                    '--controller-manager', '/controller_manager',
                    '--param-file', os.path.join(pkg_path, 'config', 'controllers.yaml'),
                ],
                output='screen',
            )
        ]
    )

    rviz = TimerAction(
        period=19.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', os.path.join(pkg_path, 'rviz', 'robot.rviz')],
                parameters=[{'use_sim_time': True}],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        disable_online_db,
        fix_model_path,
        gzserver,
        rsp,
        gzclient,
        spawn,
        load_joint_state_broadcaster,
        load_joint_trajectory_controller,
        rviz,
    ])
"""

PACKAGE_XML_TEMPLATE = """<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd"
            schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>PKG_NAME</name>
  <version>0.0.1</version>
  <description>Auto-generated ROS 2 description package for PKG_NAME</description>
  <maintainer email="todo@example.com">todo</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>gazebo_ros</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
"""

CMAKE_TEMPLATE = """cmake_minimum_required(VERSION 3.8)
project(PKG_NAME)

find_package(ament_cmake REQUIRED)

install(DIRECTORY models launch rviz config gazebo
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
"""

RVIZ_TEMPLATE = """Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
        - /RobotModel1
        - /RobotModel1/Description Topic1
        - /TF1
      Splitter Ratio: 0.5
    Tree Height: 549
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Expanded:
      - /2D Goal Pose1
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.5886790156364441
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
  - Class: rviz_common/Time
    Experimental: false
    Name: Time
    SyncMode: 0
    SyncSource: ""
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true
    - Alpha: 1
      Class: rviz_default_plugins/RobotModel
      Collision Enabled: false
      Description File: ""
      Description Source: Topic
      Description Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /robot_description
      Enabled: true
      Links:
        All Links Enabled: true
        Expand Joint Details: false
        Expand Link Details: false
        Expand Tree: false
        Link Tree Style: Links in Alphabetic Order
      Mass Properties:
        Inertia: false
        Mass: false
      Name: RobotModel
      TF Prefix: ""
      Update Interval: 0
      Value: true
      Visual Enabled: true
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: true
      Marker Scale: 0.20000000298023224
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: false
      Tree: {}
      Update Interval: 0
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: base_link
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
    - Class: rviz_default_plugins/SetInitialPose
      Covariance x: 0.25
      Covariance y: 0.25
      Covariance yaw: 0.06853891909122467
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /initialpose
    - Class: rviz_default_plugins/SetGoal
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /goal_pose
    - Class: rviz_default_plugins/PublishPoint
      Single click: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /clicked_point
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 0.9621571898460388
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 0.38039860129356384
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 4.968550205230713
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 846
  Hide Left Dock: false
  Hide Right Dock: false
  QMainWindow State: 000000ff00000000fd000000040000000000000156000002b0fc0200000008fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003d000002b0000000c900fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261000000010000010f000002b0fc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073010000003d000002b0000000a400fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000004b00000003efc0100000002fb0000000800540069006d00650100000000000004b0000002fb00fffffffb0000000800540069006d006501000000000000045000000000000000000000023f000002b000000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  Selection:
    collapsed: false
  Time:
    collapsed: false
  Tool Properties:
    collapsed: false
  Views:
    collapsed: false
  Width: 1200
  X: 499
  Y: 72
"""
