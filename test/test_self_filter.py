"""Validate measured-angle FK and actual mesh surfaces, interiors and padding."""

from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from pointcloud_filters.self_filter import _collada_mesh, SelfFilter


CUBE_VERTICES = np.array([
    [-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
    [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]], dtype=float)
CUBE_TRIANGLES = np.array([
    [0, 2, 1], [0, 3, 2], [4, 5, 6], [4, 6, 7],
    [0, 1, 5], [0, 5, 4], [1, 2, 6], [1, 6, 5],
    [2, 3, 7], [2, 7, 6], [3, 0, 4], [3, 4, 7]])


def _write_dae(path, vertices, triangles, unit=1, transform=None, up='Z_UP'):
    matrix = np.eye(4) if transform is None else transform
    path.write_text(f'''<?xml version="1.0"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
<asset><unit meter="{unit}"/><up_axis>{up}</up_axis></asset>
<library_geometries><geometry id="shape"><mesh><source id="positions">
<float_array id="data" count="{vertices.size}">{' '.join(map(str, vertices.ravel()))}</float_array>
<technique_common><accessor source="#data" count="{len(vertices)}" stride="3"/>
</technique_common></source><vertices id="v"><input semantic="POSITION" source="#positions"/>
</vertices><triangles count="{len(triangles)}"><input semantic="VERTEX" source="#v" offset="0"/>
<p>{' '.join(map(str, triangles.ravel()))}</p></triangles></mesh></geometry></library_geometries>
<library_visual_scenes><visual_scene id="scene"><node><matrix>
{' '.join(map(str, matrix.ravel()))}</matrix><instance_geometry url="#shape"/></node>
</visual_scene></library_visual_scenes><scene><instance_visual_scene url="#scene"/></scene>
</COLLADA>''')


@pytest.fixture(scope='module')
def open3d_available():
    """Require the production raycasting backend for geometry tests."""
    return pytest.importorskip('open3d')


@pytest.fixture
def articulated_model(tmp_path, open3d_available):
    """Use two named joints, a fixed origin rotation and a collision-free leaf."""
    _write_dae(tmp_path / 'box.dae', CUBE_VERTICES * [0.5, 0.2, 0.2], CUBE_TRIANGLES)
    path = tmp_path / 'test.urdf'
    path.write_text('''<robot name="test">
<link name="base_link"/><link name="turntable"/>
<link name="arm"><collision><origin xyz="1 0 0"/>
<geometry><mesh filename="box.dae"/></geometry></collision></link><link name="tip"/>
<joint name="swing" type="continuous"><parent link="base_link"/><child link="turntable"/>
<origin xyz="0 0 1"/><axis xyz="0 0 1"/></joint>
<joint name="arm_angle" type="revolute"><parent link="turntable"/><child link="arm"/>
<origin xyz="2 0 0" rpy="0 0 1.5707963267948966"/><axis xyz="0 0 1"/></joint>
<joint name="unused_tip" type="revolute"><parent link="arm"/><child link="tip"/></joint>
</robot>''')
    return SelfFilter(path, padding=0.1)


def test_collada_units_scene_transform_and_axis(tmp_path):
    """Apply node rotation/translation in source units before converting the up axis."""
    path = tmp_path / 'scene.dae'
    transform = np.eye(4)
    transform[:3, 3] = [5, 7, 11]
    transform[:3, :3] = Rotation.from_euler('z', 37, degrees=True).as_matrix()
    _write_dae(path, CUBE_VERTICES, CUBE_TRIANGLES, 0.001, transform, 'Y_UP')
    vertices, triangles = _collada_mesh(path)
    expected = (CUBE_VERTICES @ transform[:3, :3].T + [5, 7, 11]) * 0.001
    expected = expected @ Rotation.from_euler('x', 90, degrees=True).as_matrix().T
    np.testing.assert_allclose(vertices, expected)
    np.testing.assert_array_equal(triangles, CUBE_TRIANGLES)


def test_joint_names_and_rigid_chain_follow_measured_angles(articulated_model):
    """Evaluate URDF origin-before-motion order without requiring unused joints."""
    model = articulated_model
    assert model.required_joints == ('swing', 'arm_angle')
    transforms = model.link_transforms({'arm_angle': -np.pi / 2, 'swing': np.pi / 2})
    np.testing.assert_allclose(transforms['arm'][:3, 3], [0, 2, 1], atol=1e-14)
    np.testing.assert_allclose(transforms['arm'][:3, :3],
                               Rotation.from_euler('z', 90, degrees=True).as_matrix(), atol=1e-14)


def test_closed_interior_surface_padding_and_exterior(articulated_model):
    """Use true triangle distances, retaining points beyond an edge's Euclidean padding."""
    model = articulated_model
    joints = {'swing': 0, 'arm_angle': -np.pi / 2}
    # Arm mesh center is (3,0,1) after the measured joint cancels origin rotation.
    points = np.array([[3, 0, 1], [3.5, 0, 1], [3.59, 0, 1], [3.61, 0, 1],
                       [3.59, 0.29, 1], [3.55, 0.25, 1], [3, 0, np.nan]])
    mask = model.mask(points, np.eye(4), joints)
    np.testing.assert_array_equal(mask, [True, True, True, False, False, True, False])


def test_cloud_frame_pose_rotation_translation_and_original_order(articulated_model):
    """Machine pose must rotate the articulated geometry before translating it."""
    model = articulated_model
    joints = {'swing': 0, 'arm_angle': -np.pi / 2}
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_euler('xyz', [0.2, -0.3, 0.7]).as_matrix()
    pose[:3, 3] = [21395, 14034, 28]
    local = np.array([[3, 0, 1], [8, 5, 4], [3.5, 0, 1]])
    points = local @ pose[:3, :3].T + pose[:3, 3]
    original = points.copy()
    np.testing.assert_array_equal(model.mask(points, pose, joints), [True, False, True])
    np.testing.assert_array_equal(points, original)
    assert not model.mask(points, np.eye(4), joints).any()
    assert model.mask(np.empty((0, 3)), pose, joints).shape == (0,)


@pytest.mark.parametrize('joints', [{}, {'swing': 0}, {'swing': np.nan, 'arm_angle': 0}])
def test_absent_or_nonfinite_joints_are_rejected(articulated_model, joints):
    """A missing measurement must never become a zero-angle robot."""
    with pytest.raises(ValueError):
        articulated_model.mask(np.zeros((1, 3)), np.eye(4), joints)


def test_nonrigid_pose_is_rejected(articulated_model):
    """Scale cannot enter the world-to-base pose transform."""
    pose = np.eye(4)
    pose[0, 0] = 2
    with pytest.raises(ValueError, match='rigid'):
        articulated_model.mask(np.zeros((1, 3)), pose, {'swing': 0, 'arm_angle': 0})


def test_open_mesh_uses_surface_distance_without_inventing_interior(tmp_path, open3d_available):
    """An open box cannot classify its empty center using an invalid parity test."""
    _write_dae(tmp_path / 'open.dae', CUBE_VERTICES, CUBE_TRIANGLES[4:])
    path = tmp_path / 'open.urdf'
    path.write_text('''<robot name="open"><link name="base_link"><collision>
<geometry><mesh filename="open.dae"/></geometry></collision></link></robot>''')
    model = SelfFilter(path, padding=0.1)
    assert not model.collisions[0].volumes
    np.testing.assert_array_equal(model.mask(
        [[0, 0, 0], [1, 0, 0], [1.05, 0, 0], [1.2, 0, 0]], np.eye(4), {}),
        [False, True, True, False])


def test_scale_uses_each_collision_center_and_keeps_padding_in_metres(tmp_path, open3d_available):
    """An offset cube expands about x=5, and its extra shell stays exactly 0.1 m."""
    _write_dae(tmp_path / 'offset.dae', CUBE_VERTICES + [5, 0, 0], CUBE_TRIANGLES)
    path = tmp_path / 'offset.urdf'
    path.write_text('''<robot name="offset"><link name="base_link"><collision>
<geometry><mesh filename="offset.dae"/></geometry></collision></link></robot>''')
    original = SelfFilter(path, padding=.1)
    expanded = SelfFilter(path, padding=.1, scale=1.3)
    assert original.scale == 1.0 and expanded.scale == 1.3
    # Original x bounds are 4..6. Expanded bounds must be 3.7..6.3,
    # followed by the unchanged 0.1 m padding, rather than 0.13 m.
    points = np.array([[3.61, 0, 0], [3.59, 0, 0], [5, 0, 0],
                       [6.39, 0, 0], [6.41, 0, 0], [5, 1.39, 0], [5, 1.41, 0]])
    np.testing.assert_array_equal(
        expanded.mask(points, np.eye(4), {}), [True, False, True, True, False, True, False])
    assert not original.mask(points[[0, 3, 5]], np.eye(4), {}).any()


def test_scale_preserves_measured_joint_origins_and_machine_pose(articulated_model, tmp_path):
    """Shape growth must not lengthen the linkage or move its measured root."""
    original = articulated_model
    expanded = SelfFilter(tmp_path / 'test.urdf', padding=.1, scale=1.3)
    joints = {'swing': np.pi / 2, 'arm_angle': -np.pi / 2}
    for name, transform in original.link_transforms(joints).items():
        np.testing.assert_allclose(expanded.link_transforms(joints)[name], transform)
    pose = np.eye(4)
    pose[:3, :3] = Rotation.from_euler('xyz', [.2, -.3, .7]).as_matrix()
    pose[:3, 3] = [20, 40, 3]
    # The mesh center remains base (0,3,1); its local long axis is base Y.
    base_points = np.array([[0, 3.74, 1], [0, 3.76, 1], [.35, 3, 1], [.37, 3, 1]])
    points = base_points @ pose[:3, :3].T + pose[:3, 3]
    np.testing.assert_array_equal(expanded.mask(points, pose, joints), [True, False, True, False])


def test_scale_keeps_original_open_surfaces_and_does_not_invent_volume(tmp_path, open3d_available):
    """The expanded shell supplements the old shell instead of replacing it."""
    _write_dae(tmp_path / 'open.dae', CUBE_VERTICES, CUBE_TRIANGLES[4:])
    path = tmp_path / 'open.urdf'
    path.write_text('''<robot name="open"><link name="base_link"><collision>
<geometry><mesh filename="open.dae"/></geometry></collision></link></robot>''')
    original = SelfFilter(path, padding=.1)
    expanded = SelfFilter(path, padding=.1, scale=1.3)
    points = np.array([[0, 0, 0], [1, 0, 0], [1.05, 0, 0],
                       [1.3, 0, 0], [1.39, 0, 0], [1.41, 0, 0]])
    before = original.mask(points, np.eye(4), {})
    after = expanded.mask(points, np.eye(4), {})
    np.testing.assert_array_equal(after, [False, True, True, True, True, False])
    assert after[before].all()


def test_stepwise_growth_retains_previously_removed_open_faces(tmp_path, open3d_available):
    """Growing an open face from x=3.9 to x=4.5 must retain its previous shell."""
    _write_dae(tmp_path / 'open.dae', CUBE_VERTICES * [3, 1, 1], CUBE_TRIANGLES[4:])
    path = tmp_path / 'open.urdf'
    path.write_text('''<robot name="open"><link name="base_link"><collision>
<geometry><mesh filename="open.dae"/></geometry></collision></link></robot>''')
    previous = SelfFilter(path, padding=.1, scale=1.3)
    expanded = SelfFilter(path, padding=.1, scale=1.5)
    # A final-shell-only model has side faces at +/-4.5, +/-1.5;
    # x=3.9 is 0.6 m from the new long-axis face and outside its 0.1 m shell.
    # The intermediate 1.3x face must keep excluding this old return.
    points = np.array([[3.9, 0, 0], [4.5, 0, 0], [4.59, 0, 0],
                       [4.61, 0, 0], [3.45, 0, 0], [0, 0, 0]])
    np.testing.assert_array_equal(expanded.mask(points, np.eye(4), {}),
                                  [True, True, True, False, False, False])
    x, y, z = np.meshgrid(np.linspace(-5, 5, 101), [-1.3, 0., 1.3], [-.5, 0., .5])
    candidates = np.column_stack((x.ravel(), y.ravel(), z.ravel()))
    old_mask = previous.mask(candidates, np.eye(4), {})
    new_mask = expanded.mask(candidates, np.eye(4), {})
    assert new_mask[old_mask].all()
    assert np.count_nonzero(new_mask) > np.count_nonzero(old_mask)
    fractional = SelfFilter(path, padding=.01, scale=1.35)
    np.testing.assert_array_equal(
        fractional.mask([[4.05, 0, 0], [4.10, 0, 0]], np.eye(4), {}), [True, False])


@pytest.mark.parametrize('scale', [0, -.5, .99, np.nan, np.inf, -np.inf])
def test_invalid_growth_scale_is_rejected_before_loading_geometry(scale):
    """Only finite enlargement factors can configure a removal model."""
    with pytest.raises(ValueError, match='self_filter_scale'):
        SelfFilter('/nonexistent/model.urdf', scale=scale)


def test_bundled_zx200_all_mesh_vertices_follow_real_bag_joint_angles(open3d_available):
    """Check every shipped collision vertex at realistic measured machine posture."""
    model = SelfFilter(Path(__file__).parents[1] / 'robot' / 'zx200.urdf')
    joints = {'swing_joint': -3.1344, 'boom_joint': -0.7916,
              'arm_joint': 1.646, 'bucket_joint': 0.004}
    assert set(model.required_joints) == set(joints)
    assert len(model.collisions) == 5
    body = next(c for c in model.collisions if c.link == 'body_link')
    # Lamp geometry has a nested scene translation; ignoring it gives zmax=3.415835.
    assert body.upper[2] == pytest.approx(2.341788, abs=1e-6)
    transforms = model.link_transforms(joints)
    pose = np.eye(4)
    pose[:3, 3] = [20.2339, 21.5367, 3.938]
    pose[:3, :3] = Rotation.from_euler('xyz', [-0.005, -0.0075, 1.71878]).as_matrix()
    for collision in model.collisions:
        matrix = pose @ transforms[collision.link] @ collision.origin
        points = collision.vertices @ matrix[:3, :3].T + matrix[:3, 3]
        assert model.mask(points, pose, joints).all(), collision.link
