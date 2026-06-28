"""
Tests for the KinematicDAG and URDFTraits detector.

To run:
    uv run pytest tests/ -v

To test with a real URDF, drop any onshape-exported URDF at:
    tests/test_robot.urdf

The tests will auto-skip if the file is not present.
"""

from pathlib import Path

import pytest

from meshflow.detector import KinematicDAG, URDFTraits

TEST_URDF = Path(__file__).parent / "test_robot.urdf"

VALID_DRIVE_PLUGINS = {
    "libgazebo_ros_diff_drive.so",
    "libgazebo_ros_skid_steer_drive.so",
}


@pytest.mark.skipif(not TEST_URDF.exists(), reason="drop a URDF at tests/test_robot.urdf to enable")
class TestKinematicDAG:
    def test_builds_without_error(self):
        dag = KinematicDAG(TEST_URDF)
        assert dag.root_node is not None

    def test_root_is_root_type(self):
        dag = KinematicDAG(TEST_URDF)
        assert dag.root_node.joint_type == "root"

    def test_root_transform_is_identity(self):
        import numpy as np
        dag = KinematicDAG(TEST_URDF)
        assert dag.root_node.global_T.shape == (4, 4)
        assert pytest.approx(dag.root_node.global_T, abs=1e-9) == np.eye(4)

    def test_has_children(self):
        dag = KinematicDAG(TEST_URDF)
        assert len(dag.root_node.children) > 0


@pytest.mark.skipif(not TEST_URDF.exists(), reason="drop a URDF at tests/test_robot.urdf to enable")
class TestURDFTraits:
    @pytest.fixture(scope="class")
    def traits(self):
        dag = KinematicDAG(TEST_URDF)
        return URDFTraits.from_dag(dag)

    def test_drive_wheels_detected(self, traits):
        assert len(traits.drive_wheels) >= 2, (
            f"Expected at least 2 drive wheels, got {len(traits.drive_wheels)}"
        )

    def test_wheel_separation_in_range(self, traits):
        assert traits.wheel_separation > 0, "wheel_separation must be positive"
        assert traits.wheel_separation < 1.0, (
            f"wheel_separation={traits.wheel_separation} exceeds 1m — likely wrong"
        )

    def test_wheel_diameter_in_range(self, traits):
        assert traits.wheel_diameter > 0, "wheel_diameter must be positive"
        assert traits.wheel_diameter < 0.3, (
            f"wheel_diameter={traits.wheel_diameter} exceeds 0.3m — likely wrong"
        )

    def test_drive_plugin_valid(self, traits):
        assert traits.drive_plugin in VALID_DRIVE_PLUGINS, (
            f"Unexpected drive_plugin: {traits.drive_plugin}"
        )

    def test_left_right_assigned(self, traits):
        assert traits.left_wheel is not None
        assert traits.right_wheel is not None
        assert traits.left_wheel.y_position >= traits.right_wheel.y_position

    def test_no_name_based_classification(self, traits):
        """Smoke-test: wheels are classified by geometry, not names."""
        # If wheels were detected, their names should be whatever the URDF uses
        # (could be English, numbers, or any language)
        for node in traits.drive_wheels:
            assert node.joint_name, "wheel node must have a joint_name"
            assert node.link_name,  "wheel node must have a link_name"

    def test_all_links_populated(self, traits):
        assert len(traits.all_links) > 0

    def test_movable_joint_names(self, traits):
        joints = traits.movable_joint_names
        # At minimum, the drive wheel joints should be there
        wheel_joints = {w.joint_name for w in traits.drive_wheels}
        for jname in wheel_joints:
            assert jname in joints, f"drive wheel joint {jname} not in movable_joint_names"


# ---------------------------------------------------------------------------
# Unit tests that don't require a real URDF
# ---------------------------------------------------------------------------

def test_rpy_to_rot3_identity():
    """Zero rotation should give identity matrix."""
    import numpy as np
    from meshflow.detector import _rpy_to_rot3
    R = _rpy_to_rot3(0.0, 0.0, 0.0)
    assert pytest.approx(R, abs=1e-9) == np.eye(3)


def test_make_transform_translation_only():
    """Pure translation with zero rotation."""
    import numpy as np
    from meshflow.detector import _make_transform
    T = _make_transform([1.0, 2.0, 3.0], [0.0, 0.0, 0.0])
    assert T[0, 3] == pytest.approx(1.0)
    assert T[1, 3] == pytest.approx(2.0)
    assert T[2, 3] == pytest.approx(3.0)
    assert pytest.approx(T[:3, :3], abs=1e-9) == np.eye(3)


def test_parse_vec3():
    from meshflow.detector import _parse_vec3
    assert _parse_vec3("1.0 2.0 3.0") == pytest.approx([1.0, 2.0, 3.0])
    assert _parse_vec3("") == pytest.approx([0.0, 0.0, 0.0])
    assert _parse_vec3(None) == pytest.approx([0.0, 0.0, 0.0])


def test_sensor_plugins_structure():
    """SENSOR_PLUGINS entries all have required keys."""
    from meshflow.detector import SENSOR_PLUGINS
    required = {"match", "plugin_file", "sensor_type", "defaults"}
    for name, config in SENSOR_PLUGINS.items():
        missing = required - set(config.keys())
        assert not missing, f"SENSOR_PLUGINS['{name}'] missing keys: {missing}"
        assert callable(config["match"]), f"match for '{name}' must be callable"
