"""
KinematicDAG — builds a spatial tree from a URDF.
URDFTraits   — classifies every node using ONLY geometry/kinematics.

ZERO name-based classification anywhere in this module.
"""

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional
from xml.etree import ElementTree as ET

import numpy as np

try:
    import trimesh as _trimesh
    _TRIMESH_AVAILABLE = True
except ImportError:
    _trimesh = None
    _TRIMESH_AVAILABLE = False


# ---------------------------------------------------------------------------
# Math helpers
# ---------------------------------------------------------------------------

def _parse_vec3(s: str, default: str = "0 0 0") -> list[float]:
    parts = (s or default).strip().split()
    return [float(x) for x in parts]


def _rpy_to_rot3(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """3×3 rotation matrix from RPY (extrinsic XYZ = R_z @ R_y @ R_x)."""
    cr, sr = math.cos(roll),  math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)
    return np.array([
        [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [-sp,    cp*sr,              cp*cr            ],
    ])


def _make_transform(xyz: list[float], rpy: list[float]) -> np.ndarray:
    """Build a 4×4 homogeneous transform from [x,y,z] and [r,p,y]."""
    T = np.eye(4)
    T[:3, :3] = _rpy_to_rot3(*rpy)
    T[:3, 3]  = xyz
    return T


# ---------------------------------------------------------------------------
# KinematicNode
# ---------------------------------------------------------------------------

@dataclass(eq=False)
class KinematicNode:
    link_name:  str
    joint_name: str
    joint_type: str        # 'continuous','fixed','revolute','prismatic','root'
    axis_world: np.ndarray # [x,y,z] rotation axis in world frame
    global_T:   np.ndarray # 4×4 transform: root → this link origin
    stl_path:   Optional[Path]
    children:   list = field(default_factory=list)
    mass:       float = 0.0
    izz:        float = 0.0  # used for inertia-fallback wheel radius

    @property
    def y_position(self) -> float:
        return float(self.global_T[1, 3])

    @property
    def z_min(self) -> float:
        """Lowest z vertex of the mesh in world frame."""
        if _TRIMESH_AVAILABLE and self.stl_path is not None and self.stl_path.exists():
            try:
                mesh = _trimesh.load(str(self.stl_path), force='mesh')
                verts = np.hstack([mesh.vertices, np.ones((len(mesh.vertices), 1))])
                world_verts = (self.global_T @ verts.T).T
                return float(world_verts[:, 2].min())
            except Exception:
                pass
        # fallback: joint z-position minus an estimated half-size
        return float(self.global_T[2, 3]) - 0.1

    @property
    def aabb_extents(self) -> Optional[np.ndarray]:
        """[dx, dy, dz] of the world-frame axis-aligned bounding box."""
        if _TRIMESH_AVAILABLE and self.stl_path is not None and self.stl_path.exists():
            try:
                mesh = _trimesh.load(str(self.stl_path), force='mesh')
                verts = np.hstack([mesh.vertices, np.ones((len(mesh.vertices), 1))])
                world_verts = (self.global_T @ verts.T).T[:, :3]
                return world_verts.max(axis=0) - world_verts.min(axis=0)
            except Exception:
                pass
        return None


# ---------------------------------------------------------------------------
# KinematicDAG
# ---------------------------------------------------------------------------

class KinematicDAG:
    """Directed kinematic graph parsed from a URDF file."""

    def __init__(self, urdf_path: Path, mesh_dir: Optional[Path] = None):
        self.urdf_path = urdf_path
        tree = ET.parse(urdf_path)
        self.xml_root = tree.getroot()

        # Default: STLs live two levels up from the URDF (models/meshes/)
        self.mesh_dir = mesh_dir or (urdf_path.parent.parent / "meshes")

        # Find the root link: the one that is never a joint child
        all_links   = {l.get('name') for l in self.xml_root.findall('link')}
        child_links = set()
        for j in self.xml_root.findall('joint'):
            c = j.find('child')
            if c is not None:
                child_links.add(c.get('link'))
        root_links = all_links - child_links

        if not root_links:
            raise ValueError("URDF has no root link (every link is a child — cycle?)")
        if len(root_links) > 1:
            # Pick alphabetically for determinism; warn
            print(f"  [WARN] Multiple root candidates {root_links}, using first alphabetically.")
        root_link_name = sorted(root_links)[0]

        self.root_node = self._build_node(
            link_name=root_link_name,
            joint_name="root",
            joint_type="root",
            joint_elem=None,
            parent_T=np.eye(4),
        )

    # ── internal helpers ────────────────────────────────────────────────────

    def _joint_transform(self, joint_elem: ET.Element) -> np.ndarray:
        origin = joint_elem.find("origin")
        if origin is None:
            return np.eye(4)
        xyz = _parse_vec3(origin.get("xyz", "0 0 0"))
        rpy = _parse_vec3(origin.get("rpy", "0 0 0"))
        return _make_transform(xyz, rpy)

    def _joint_axis_world(self, joint_elem: Optional[ET.Element], rot3: np.ndarray) -> np.ndarray:
        if joint_elem is None:
            return np.array([0.0, 0.0, 1.0])
        ax_el = joint_elem.find("axis")
        if ax_el is None:
            local = np.array([0.0, 0.0, 1.0])
        else:
            local = np.array(_parse_vec3(ax_el.get("xyz", "0 0 1")))
        norm = np.linalg.norm(local)
        if norm > 1e-9:
            local = local / norm
        return rot3 @ local

    def _stl_path_for_link(self, link_elem: Optional[ET.Element]) -> Optional[Path]:
        if link_elem is None:
            return None
        mesh_el = link_elem.find("visual/geometry/mesh")
        if mesh_el is None:
            return None
        raw = mesh_el.get("filename", "")
        bare = raw.split("/")[-1]
        if not bare:
            return None
        candidate = self.mesh_dir / bare
        return candidate if candidate.exists() else None

    def _link_elem(self, name: str) -> Optional[ET.Element]:
        for l in self.xml_root.findall('link'):
            if l.get('name') == name:
                return l
        return None

    def _build_node(
        self,
        link_name:  str,
        joint_name: str,
        joint_type: str,
        joint_elem: Optional[ET.Element],
        parent_T:   np.ndarray,
    ) -> KinematicNode:
        # Accumulate transform
        if joint_elem is not None:
            global_T = parent_T @ self._joint_transform(joint_elem)
        else:
            global_T = parent_T.copy()

        rot3       = global_T[:3, :3]
        axis_world = self._joint_axis_world(joint_elem, rot3)
        link_el    = self._link_elem(link_name)
        stl_path   = self._stl_path_for_link(link_el)

        # Extract inertial data for radius fallback
        mass = izz = 0.0
        if link_el is not None:
            inertial = link_el.find('inertial')
            if inertial is not None:
                m_el = inertial.find('mass')
                i_el = inertial.find('inertia')
                if m_el is not None:
                    mass = float(m_el.get('value', 0))
                if i_el is not None:
                    izz = float(i_el.get('izz', 0))

        node = KinematicNode(
            link_name=link_name,
            joint_name=joint_name,
            joint_type=joint_type,
            axis_world=axis_world,
            global_T=global_T,
            stl_path=stl_path,
            mass=mass,
            izz=izz,
        )

        # Recurse into children
        for j in self.xml_root.findall('joint'):
            p_el = j.find('parent')
            c_el = j.find('child')
            if p_el is None or c_el is None:
                continue
            if p_el.get('link') == link_name:
                child_node = self._build_node(
                    link_name=c_el.get('link', ''),
                    joint_name=j.get('name', ''),
                    joint_type=j.get('type', 'fixed'),
                    joint_elem=j,
                    parent_T=global_T,
                )
                node.children.append(child_node)

        return node


# ---------------------------------------------------------------------------
# Sensor geometry heuristics (trimesh-gated — NO name matching)
# ---------------------------------------------------------------------------

def _looks_like_lidar(node: KinematicNode) -> bool:
    """Flat disc/cylinder: the two XZ dimensions dwarf the Y (thickness)."""
    ext = node.aabb_extents
    if ext is None:
        return False
    xz = sorted([float(ext[0]), float(ext[2])])
    return xz[1] > 0.04 and xz[0] / xz[1] < 0.5


def _looks_like_imu(node: KinematicNode) -> bool:
    """Tiny cube: all extents under 40 mm."""
    ext = node.aabb_extents
    if ext is None:
        return False
    return all(float(e) < 0.04 for e in ext)


def _looks_like_camera(node: KinematicNode) -> bool:
    """Thin rectangular box (not disc-like, not huge)."""
    ext = node.aabb_extents
    if ext is None:
        return False
    s = sorted([float(e) for e in ext])
    return s[0] < 0.04 and s[-1] < 0.15 and not _looks_like_lidar(node)


def _looks_like_depth(node: KinematicNode) -> bool:
    """Depth camera: like a camera but the largest dim ≥ 80 mm."""
    ext = node.aabb_extents
    if ext is None:
        return False
    s = sorted([float(e) for e in ext])
    return s[0] < 0.06 and s[-1] >= 0.08 and not _looks_like_lidar(node)


# ---------------------------------------------------------------------------
# Sensor plugin registry
# ---------------------------------------------------------------------------

SENSOR_PLUGINS: dict = {
    "lidar_revolute": {
        "match": lambda node: node.joint_type == "revolute",
        "plugin_file": "libgazebo_ros_ray_sensor.so",
        "sensor_type": "ray",
        "defaults": {
            "samples": 360, "min_angle": -3.14159, "max_angle": 3.14159,
            "range_min": 0.12, "range_max": 12.0, "update_rate": 20,
        },
    },
    "lidar_fixed": {
        "match": lambda node: node.joint_type == "fixed" and _looks_like_lidar(node),
        "plugin_file": "libgazebo_ros_ray_sensor.so",
        "sensor_type": "ray",
        "defaults": {
            "samples": 360, "min_angle": -3.14159, "max_angle": 3.14159,
            "range_min": 0.12, "range_max": 12.0, "update_rate": 20,
        },
    },
    "camera_rgb": {
        "match": lambda node: node.joint_type == "fixed" and _looks_like_camera(node),
        "plugin_file": "libgazebo_ros_camera.so",
        "sensor_type": "camera",
        "defaults": {"width": 640, "height": 480, "fov": 1.047, "update_rate": 30},
    },
    "depth_camera": {
        "match": lambda node: node.joint_type == "fixed" and _looks_like_depth(node),
        "plugin_file": "libgazebo_ros_depth_camera.so",
        "sensor_type": "depth",
        "defaults": {"width": 640, "height": 480, "fov": 1.047, "update_rate": 30},
    },
    "imu": {
        "match": lambda node: node.joint_type == "fixed" and _looks_like_imu(node),
        "plugin_file": "libgazebo_ros_imu_sensor.so",
        "sensor_type": "imu",
        "defaults": {"update_rate": 100},
    },
}


# ---------------------------------------------------------------------------
# URDFTraits
# ---------------------------------------------------------------------------

def _is_leaf_or_fixed_children(node: KinematicNode) -> bool:
    if not node.children:
        return True
    return all(c.joint_type == 'fixed' for c in node.children)


def _compute_wheel_diameter(wheel: Optional[KinematicNode]) -> float:
    """Diameter via trimesh AABB → inertia formula → 0.060 default."""
    if wheel is None:
        return 0.060

    # Priority 1: trimesh AABB (wheel rotates about Y → diameter = max(X,Z))
    ext = wheel.aabb_extents
    if ext is not None:
        d = float(max(ext[0], ext[2]))
        if d > 0.01:
            return round(d, 4)

    # Priority 2: inertia tensor  izz = 0.5 * m * r²
    if wheel.mass > 0 and wheel.izz > 0:
        r = math.sqrt(2 * wheel.izz / wheel.mass)
        return round(r * 2, 4)

    # Priority 3: default
    print("  [WARN] Could not determine wheel diameter from geometry or inertia. Using 0.060 m.")
    return 0.060


@dataclass
class URDFTraits:
    drive_wheels:     list   # sorted y ascending (drive_wheels[0]=rightmost, [-1]=leftmost)
    passive_contacts: list
    sensors:          list
    all_links:        list
    left_wheel:       Optional[KinematicNode]
    right_wheel:      Optional[KinematicNode]
    wheel_separation: float
    wheel_diameter:   float
    drive_plugin:     str    # libgazebo_ros_diff_drive.so or ..._skid_steer_drive.so

    @property
    def movable_joint_names(self) -> list[str]:
        """Joint names for the joint_state_publisher plugin."""
        joints = [w.joint_name for w in self.drive_wheels]
        joints += [s.joint_name for s in self.sensors if s.joint_type == 'revolute']
        return joints

    @classmethod
    def from_dag(cls, dag: KinematicDAG) -> 'URDFTraits':
        drive_wheels:     list[KinematicNode] = []
        passive_contacts: list[KinematicNode] = []
        sensors:          list[KinematicNode] = []
        root_link = dag.root_node.link_name

        Y_HAT = np.array([0.0, 1.0, 0.0])

        def _classify(node: KinematicNode, parent_is_wheel: bool = False,
                      siblings: list | None = None) -> None:
            if node.joint_type == 'root':
                for child in node.children:
                    _classify(child, siblings=node.children)
                return

            jt     = node.joint_type
            z_min  = node.z_min
            y_dot  = abs(float(np.dot(node.axis_world, Y_HAT)))

            is_drive_wheel = (jt == 'continuous' and y_dot > 0.85 and z_min < 0.20)

            def _has_colocated_wheel_sibling() -> bool:
                if not siblings:
                    return False
                pos = node.global_T[:3, 3]
                return any(
                    s is not node
                    and s.joint_type == 'continuous'
                    and abs(float(np.dot(s.axis_world, Y_HAT))) > 0.85
                    and float(np.linalg.norm(s.global_T[:3, 3] - pos)) < 0.015
                    for s in siblings
                )

            # ── Drive wheel ────────────────────────────────────────────────
            if is_drive_wheel:
                drive_wheels.append(node)

            # ── Passive contact (caster) ───────────────────────────────────
            elif jt == 'fixed' and not parent_is_wheel and node.link_name != root_link and z_min < 0.01:
                passive_contacts.append(node)

            # ── Rotating sensor (revolute lidar) ──────────────────────────
            elif jt == 'revolute' and y_dot < 0.5:
                sensors.append(node)

            # ── Fixed sensor (camera / IMU / depth / fixed lidar) ─────────
            # parent_is_wheel: excludes fixed children of drive wheels (sample_robo-style hubs)
            # _has_colocated_wheel_sibling: excludes fixed nodes co-located with a wheel sibling
            #   (sam-style hubs where both hub and wheel are direct children of base_link)
            elif (jt == 'fixed' and not parent_is_wheel and not _has_colocated_wheel_sibling()
                  and z_min > 0.05 and node.link_name != root_link
                  and _is_leaf_or_fixed_children(node)):
                sensors.append(node)

            else:
                if jt not in ('fixed',):
                    print(f"  [INFO] Unclassified node: {node.link_name} (type={jt}, z_min={z_min:.3f})")

            for child in node.children:
                _classify(child, parent_is_wheel=is_drive_wheel, siblings=node.children)

        _classify(dag.root_node)

        if not _TRIMESH_AVAILABLE:
            if sensors:
                fixed_sensors = [s for s in sensors if s.joint_type == 'fixed']
                if fixed_sensors:
                    print("  [WARN] trimesh not installed — fixed sensor types cannot be classified by geometry.")
                    print("         Install trimesh for camera/IMU/depth detection. Revolute sensors (lidar) still work.")

        # Sort drive wheels by y_position (ascending → [0]=right, [-1]=left)
        drive_wheels.sort(key=lambda n: n.y_position)

        left_wheel  = drive_wheels[-1] if drive_wheels else None
        right_wheel = drive_wheels[0]  if drive_wheels else None

        wheel_separation = (
            abs(left_wheel.y_position - right_wheel.y_position)
            if left_wheel is not None and right_wheel is not None
            else 0.0
        )
        wheel_separation = round(wheel_separation, 4)
        wheel_diameter   = _compute_wheel_diameter(left_wheel)

        n = len(drive_wheels)
        if n == 4:
            drive_plugin = "libgazebo_ros_skid_steer_drive.so"
        elif n > 4:
            drive_plugin = "libgazebo_ros_diff_drive.so"
            print(f"  [WARN] {n} drive wheels detected — using outer pair with diff_drive plugin.")
        else:
            drive_plugin = "libgazebo_ros_diff_drive.so"
            if n < 2:
                print(f"  [WARN] Only {n} drive wheel(s) detected — plugin params may be incomplete.")

        all_links = [l.get('name') for l in dag.xml_root.findall('link')]

        print(f"  Detected {n} drive wheel(s), {len(passive_contacts)} caster(s), {len(sensors)} sensor(s).")
        if left_wheel and right_wheel:
            print(f"  wheel_separation={wheel_separation} m  wheel_diameter={wheel_diameter} m")
            print(f"  drive_plugin: {drive_plugin}")

        return cls(
            drive_wheels=drive_wheels,
            passive_contacts=passive_contacts,
            sensors=sensors,
            all_links=all_links,
            left_wheel=left_wheel,
            right_wheel=right_wheel,
            wheel_separation=wheel_separation,
            wheel_diameter=wheel_diameter,
            drive_plugin=drive_plugin,
        )
