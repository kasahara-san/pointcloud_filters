#!/usr/bin/env python3
"""Fit and validate an onboard mounting pose using sampled rosbag2 SQLite frames."""

import argparse
from copy import deepcopy
from dataclasses import asdict
import json
from pathlib import Path
import sqlite3
import time

import numpy as np
from rclpy.serialization import deserialize_message
from scipy.optimize import least_squares
from sensor_msgs.msg import JointState, PointCloud2
from geometry_msgs.msg import PoseStamped
import yaml

from pointcloud_filters.cloud_io import read_cloud, select_cloud
from pointcloud_filters.filtering import FilterParameters, filter_indices
from pointcloud_filters.meshing import MeshConverter
from pointcloud_filters.onboard import OnboardGeometry, synchronized_header
from pointcloud_filters.position import model_origin_matrix
from pointcloud_filters.self_filter import SelfFilter
from pointcloud_filters.state_history import stamp_ns, StateHistory


def load_samples(bag, config, model, count):
    """Read only relevant state payloads and evenly spaced cloud payloads."""
    path = Path(bag)
    files = sorted(path.glob('*.db3')) if path.is_dir() else [path]
    if len(files) != 1 or files[0].suffix != '.db3':
        raise ValueError('Provide a single .db3 or a directory with one SQLite bag file')
    connection = sqlite3.connect(files[0].resolve().as_uri() + '?mode=ro', uri=True)
    try:
        topics = {name: identity for identity, name in connection.execute(
            'SELECT id,name FROM topics')}
        names = [config[key] for key in ('input_topic', 'pose_topic', 'joint_states_topic')]
        identities = [topics[name] for name in names]
        rows = connection.execute(
            'SELECT id,timestamp,topic_id FROM messages WHERE topic_id IN (?,?,?)',
            identities).fetchall()
        clouds = [(identity, timestamp) for identity, timestamp, topic in rows
                  if topic == identities[0]]
        history = StateHistory(duration_sec=1e6)
        aliases = dict(entry.split(':', 1) for entry in config.get('joint_name_map', [])
                       if entry)
        pose_times, joint_times = [], {}
        for identity, cls in zip(identities[1:], (PoseStamped, JointState)):
            for (payload,) in connection.execute(
                    'SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp', (identity,)):
                message = deserialize_message(payload, cls)
                if cls == PoseStamped:
                    history.add_pose(message)
                    pose_times.append(stamp_ns(message.header.stamp))
                else:
                    message.name = [aliases.get(name, name) for name in message.name]
                    history.add_joints(message)
                    for name in message.name:
                        joint_times.setdefault(name, []).append(stamp_ns(message.header.stamp))
        result = []
        if not clouds:
            raise ValueError('No input point clouds in the recording')
        for index in np.unique(np.linspace(0, len(clouds) - 1, count).astype(int)):
            identity, recorded = clouds[index]
            (payload,) = connection.execute(
                'SELECT data FROM messages WHERE id=?', (int(identity),)).fetchone()
            cloud = deserialize_message(payload, PointCloud2)
            header = synchronized_header(cloud.header, config.get('cloud_time_offset_sec', 0.))
            timestamp = stamp_ns(header.stamp)
            pose = history.pose_at(timestamp, config['max_pose_time_diff_sec'])
            joints = history.joints_at(
                timestamp, model.required_joints, config['max_joint_time_diff_sec'])
            result.append(dict(
                index=int(index), recorded_ns=recorded, cloud=cloud, pose=pose, joints=joints,
                pose_dt_sec=min(abs(t - timestamp) for t in pose_times) / 1e9,
                joint_dt_sec=max(min(abs(t - timestamp) for t in joint_times[name])
                                 for name in model.required_joints) / 1e9))
        return result, len(clouds)
    finally:
        connection.close()


def surface_distances(points, matrix, model, joints):
    """Compare observed returns to CAD triangles, without sampling hidden surfaces."""
    transforms = model.link_transforms(joints)
    distances = []
    for collision in model.collisions:
        cloud_from_mesh = matrix @ transforms[collision.link] @ collision.origin
        local = (points - cloud_from_mesh[:3, 3]) @ cloud_from_mesh[:3, :3]
        query = collision._o3d.core.Tensor(local.astype(np.float32))
        distances.append(collision.scene.compute_distance(query, nthreads=2).numpy())
    return np.min(distances, axis=0).astype(float)


def fit_mount(samples, model):
    """Fit visible elevated machine returns; retain other frames as holdouts."""
    training_indices = sorted({0, len(samples) // 2, len(samples) - 1})
    selected = []
    for index in training_indices:
        points, _ = read_cloud(samples[index]['cloud'])
        # Dataset-specific ROI in the observed sideways Hesai axes. Excludes ground.
        valid = (np.isfinite(points).all(axis=1) & (np.linalg.norm(points, axis=1) > .5)
                 & (abs(points[:, 0]) < 2.) & (points[:, 1] > .25)
                 & (points[:, 1] < 2.5) & (points[:, 2] > -.5) & (points[:, 2] < 3.4))
        points = points[valid]
        _, indices = np.unique(np.floor(points / .08), axis=0, return_index=True)
        if not len(indices):
            raise ValueError('No machine returns inside the calibration ROI')
        selected.append((points[indices], samples[index]['joints']))

    def residual(parameters):
        cloud_from_base = np.linalg.inv(model_origin_matrix(parameters[:3], parameters[3:]))
        return np.concatenate([surface_distances(points, cloud_from_base, model, joints)
                               for points, joints in selected])

    fits = []
    for initial_x in (-.2, .5, 1.2):
        fit = least_squares(
            residual, [initial_x, 0., 2.85, -np.pi / 2, 0., -np.pi / 2],
            bounds=([-.8, -.5, 2.65, -1.65, -.08, -1.75],
                    [1.8, .5, 3.05, -1.49, .08, -1.39]),
            loss='soft_l1', f_scale=.07, diff_step=.0005, max_nfev=100)
        fits.append(dict(xyz=fit.x[:3].tolist(), rpy=fit.x[3:].tolist(), cost=float(fit.cost),
                         success=bool(fit.success), evaluations=int(fit.nfev)))
    return dict(training_frames=[samples[i]['index'] for i in training_indices],
                method='Bounded observed-point-to-triangle soft_l1 fit, three initializations',
                best=min(fits, key=lambda entry: entry['cost']), fits=fits)


def validate(samples, config, model, check_mesh):
    """Check byte preservation, radius, zero returns, self exclusion and mesh output."""
    mount = OnboardGeometry(config['sensor_frame'], config['sensor_xyz'], config['sensor_rpy'],
                            config['min_sensor_range_m'])
    parameters = FilterParameters(**{
        key: config.get(key, value) for key, value in asdict(FilterParameters()).items()})
    origin = model_origin_matrix(config['model_origin_xyz'], config['model_origin_rpy'])
    rows = []
    for sample in samples:
        started = time.monotonic()
        cloud = sample['cloud']
        mount.check_frame(cloud.header)
        xyz, records = read_cloud(cloud)
        offsets = mount.map_offsets(xyz, sample['pose'], config['pose_frame'])
        valid = mount.valid_returns(xyz)
        offsets[~valid] = np.nan
        candidates = np.flatnonzero(valid & (np.linalg.norm(offsets[:, :2], axis=1)
                                             <= parameters.radius_m))
        mask = np.zeros(len(xyz), dtype=bool)
        matrix = mount.cloud_from_base @ origin
        mask[candidates] = model.mask(xyz[candidates], matrix, sample['joints'])
        indices, stats = filter_indices(offsets, [0., 0.], parameters, mask)
        output = select_cloud(cloud, records, indices)
        assert len(indices) > 0
        assert output.header == cloud.header and output.fields == cloud.fields
        assert bytes(output.data) == records[indices].tobytes()
        assert np.isfinite(xyz[indices]).all() and valid[indices].all()
        assert np.all(np.linalg.norm(offsets[indices, :2], axis=1) <= parameters.radius_m)
        assert not model.mask(xyz[indices], matrix, sample['joints']).any()
        base_points = xyz @ mount.base_from_cloud[:3, :3].T + mount.base_from_cloud[:3, 3]
        # A geometric proxy, not labelled terrain: points well outside the chassis.
        ground = (valid & (np.abs(base_points[:, 2]) < .25)
                  & (np.linalg.norm(base_points[:, :2], axis=1) > 3.5)
                  & (np.linalg.norm(offsets[:, :2], axis=1) <= parameters.radius_m))
        # Enlarged blade margins can legitimately extend beyond the 3.5 m band.
        # Report those nearby losses; terrain clearly beyond the model must survive.
        far_ground = ground & (np.linalg.norm(base_points[:, :2], axis=1) > 4.5)
        assert not np.any(mask & far_ground)
        entry = dict(index=sample['index'], pose_dt_sec=sample['pose_dt_sec'],
                     joint_dt_sec=sample['joint_dt_sec'], **stats,
                     ground_proxy_points=int(ground.sum()),
                     ground_proxy_self_removed=int(np.count_nonzero(mask & ground)),
                     far_ground_proxy_points=int(far_ground.sum()),
                     far_ground_proxy_self_removed=int(np.count_nonzero(mask & far_ground)),
                     processing_sec=time.monotonic() - started)
        if check_mesh and sample in (samples[0], samples[len(samples) // 2], samples[-1]):
            mesh, mesh_stats = MeshConverter(config['mesh_resolution']).convert(
                xyz[indices], excluded=lambda p: model.mask(p, matrix, sample['joints']),
                exclusion_step=min(.075, max(.01, model.padding / 2)))
            assert mesh.triangles
            assert not model.mask(np.array([[p.x, p.y, p.z] for p in mesh.vertices]),
                                  matrix, sample['joints']).any()
            entry['mesh'] = mesh_stats
        rows.append(entry)
        print(json.dumps(entry), flush=True)
    return rows


def main():
    """Expose recording/profile overrides and optional repeated mounting estimation."""
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True)
    parser.add_argument('--params', default=str(root / 'config/d37pxi.yaml'))
    parser.add_argument('--samples', type=int, default=15)
    parser.add_argument('--report', default='/tmp/d37pxi_validation.json')
    parser.add_argument('--fit', action='store_true')
    parser.add_argument('--check-mesh', action='store_true')
    args = parser.parse_args()
    if args.samples < 3:
        parser.error('--samples must be >= 3')
    config = yaml.safe_load(Path(args.params).read_text())['pointcloud_filter']['ros__parameters']
    model = SelfFilter(root / 'robot/d37pxi/d37pxi.urdf',
                       padding=config['self_filter_padding'], scale=config['self_filter_scale'],
                       geometry=config.get('self_filter_geometry', 'mesh'))
    samples, count = load_samples(args.bag, config, model, args.samples)
    report = dict(bag=args.bag, total_clouds=count, params=deepcopy(config))
    if args.fit:
        # Extrinsic calibration always fits observed CAD surfaces, not the safety envelope.
        cad = SelfFilter(root / 'robot/d37pxi/d37pxi.urdf')
        report['calibration'] = fit_mount(samples, cad)
    report['validation'] = validate(samples, config, model, args.check_mesh)
    report['limitations'] = [
        'Mounting pose and sensor clock offset are provisional and recording-specific.',
        'Ground retention uses a geometric proxy; there are no manually labelled points.',
        'Hidden regions remain unobserved; points are not accumulated across moving scans.']
    Path(args.report).write_text(json.dumps(report, indent=2) + '\n')
    print(f'Passed {len(samples)} sampled frames; report: {args.report}', flush=True)


if __name__ == '__main__':
    main()
