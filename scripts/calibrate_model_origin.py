#!/usr/bin/env python3
"""
Estimate a provisional CAD-root transform from body/crawler surface agreement.

This is an offline, bag-specific estimate. It never changes the measured GNSS
pose or the package configuration, and cannot replace surveyed calibration.
The first frame trains the fit; middle/last frames are independent checks.
"""

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from tf2_ros import Buffer
import yaml

from pointcloud_filters.bag import resolve_bag_path
from pointcloud_filters.cloud_io import read_cloud
from pointcloud_filters.position import model_origin_matrix, pose_matrix_in_cloud_frame
from pointcloud_filters.self_filter import SelfFilter
from pointcloud_filters.state_history import stamp_ns, StateHistory
from validate_bag import _messages


def _samples(bag_path, config, model):
    pose_topic = config.get('pose_topic', '/zx200/global_pose')
    joint_topic = config.get('joint_states_topic', '/zx200/joint_states')
    cloud_topic = config.get('input_topic', '/merged_pointcloud')
    history, tf_buffer, timestamps = StateHistory(duration_sec=1e9), Buffer(), []
    for topic, message, _ in _messages(
            bag_path, [pose_topic, joint_topic, cloud_topic, '/tf_static']):
        if topic == pose_topic:
            history.add_pose(message)
        elif topic == joint_topic:
            history.add_joints(message)
        elif topic == '/tf_static':
            for transform in message.transforms:
                tf_buffer.set_transform_static(transform, 'model_calibration')
        else:
            timestamps.append(stamp_ns(message.header.stamp))
    if not timestamps:
        raise ValueError(f'No clouds on {cloud_topic}')
    wanted = {0, len(timestamps) // 2, len(timestamps) - 1}
    result = []
    for index, (_, cloud, _) in enumerate(_messages(bag_path, [cloud_topic])):
        if index not in wanted:
            continue
        timestamp = stamp_ns(cloud.header.stamp)
        pose = history.pose_at(timestamp, config.get('max_pose_time_diff_sec', 0.5))
        joints = history.joints_at(
            timestamp, model.required_joints, config.get('max_joint_time_diff_sec', 0.5))
        matrix = pose_matrix_in_cloud_frame(
            pose, cloud.header, tf_buffer, config.get('max_pose_time_diff_sec', 0.5),
            config.get('pose_frame', 'world'))
        points, _ = read_cloud(cloud)
        finite = np.isfinite(points).all(axis=1)
        points = points[finite & (np.linalg.norm(points[:, :2] - matrix[:2, 3], axis=1) < 10)]
        result.append({'index': index, 'stamp_ns': timestamp, 'frame': cloud.header.frame_id,
                       'points': points, 'matrix': matrix, 'joints': joints,
                       'links': model.link_transforms(joints)})
    return result


def _transform(parameters):
    return model_origin_matrix(parameters[:3], [0., 0., parameters[3]])


def _distances(points, sample, transform, collisions):
    if not len(points):
        raise ValueError('The selected calibration region contains no points')
    distances = []
    for collision in collisions:
        matrix = sample['matrix'] @ transform @ sample['links'][collision.link] @ collision.origin
        local = (points - matrix[:3, 3]) @ matrix[:3, :3]
        query = collision._o3d.core.Tensor(local.astype(np.float32))
        distances.append(collision.scene.compute_distance(query, nthreads=2).numpy())
    return np.min(distances, axis=0).astype(float)


def _summary(distances):
    return {'count': len(distances), 'median_m': float(np.median(distances)),
            'p90_m': float(np.percentile(distances, 90)),
            'clipped_mean_m': float(np.minimum(distances, 0.5).mean()),
            'within_015m': int(np.count_nonzero(distances <= 0.15)),
            'within_030m': int(np.count_nonzero(distances <= 0.30))}


def _preview(path, samples, model, transform, padding):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    figure, axes = plt.subplots(len(samples), 2, figsize=(12, 5 * len(samples)), squeeze=False)
    model.padding = padding
    for row, sample in enumerate(samples):
        points, base = sample['points'], sample['matrix']
        points = points[np.linalg.norm(points[:, :2] - base[:2, 3], axis=1) < 4]
        matrix = base @ transform
        mask = model.mask(points, matrix, sample['joints'])
        for axis, (horizontal, vertical) in zip(axes[row], [(0, 1), (0, 2)]):
            axis.scatter(points[:, horizontal], points[:, vertical], c='0.5', s=5,
                         label='Recorded points')
            axis.scatter(points[mask, horizontal], points[mask, vertical], c='red', s=7,
                         label='Removed at configured padding')
            for collision in model.collisions:
                local = matrix @ sample['links'][collision.link] @ collision.origin
                vertices = collision.vertices @ local[:3, :3].T + local[:3, 3]
                axis.scatter(vertices[:, horizontal], vertices[:, vertical], s=0.3,
                             alpha=0.18, label=collision.link)
            axis.scatter(base[horizontal, 3], base[vertical, 3], c='black', marker='+',
                         s=100, label='Measured GNSS base')
            axis.set_title(f'Frame {sample["index"]}: {"XYZ"[horizontal]}{"XYZ"[vertical]}')
            axis.set_xlabel('XYZ'[horizontal] + ' [m]')
            axis.set_ylabel('XYZ'[vertical] + ' [m]')
            axis.set_aspect('equal')
            axis.grid()
            axis.set_xlim(base[horizontal, 3] - 4, base[horizontal, 3] + 4)
            if vertical == 2:
                axis.set_ylim(-0.5, 6)
    axes[0, 0].legend(fontsize=6, markerscale=2)
    figure.suptitle('Configured provisional CAD origin; independent first/middle/last frames')
    figure.tight_layout()
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=160)
    plt.close(figure)


def calibrate(args):
    """Fit the raw measured base to model transform and validate separate frames."""
    root = Path(__file__).resolve().parents[1]
    config_path = Path(args.params) if args.params else root / 'config/pointcloud_filter.yaml'
    config = yaml.safe_load(config_path.read_text())['pointcloud_filter']['ros__parameters']
    path = config.get('robot_description_path') or root / 'robot/zx200.urdf'
    model = SelfFilter(path)
    bag_path = resolve_bag_path(args.bag)
    samples = _samples(bag_path, config, model)
    collisions = [collision for collision in model.collisions
                  if collision.link in ('body_link', 'track_link')]
    if len(collisions) != 2:
        raise ValueError('Calibration requires ZX200 body_link and track_link collision meshes')
    training = samples[0]
    points = training['points']
    distances_xy = np.linalg.norm(points[:, :2] - training['matrix'][:2, 3], axis=1)
    selection = (distances_xy < args.radius) & (points[:, 2] > args.min_height)
    training_points = points[selection]

    def residual(parameters):
        return _distances(training_points, training, _transform(parameters), collisions)

    fits = []
    for initial_z in args.initial_z:
        fit = least_squares(
            residual, [0., 0., initial_z, 0.],
            bounds=([-1., -1., args.z_range[0], -0.5], [1., 1., args.z_range[1], 0.5]),
            loss='soft_l1', f_scale=0.15, diff_step=0.01, max_nfev=120, xtol=1e-6)
        entry = {'initial_z_m': initial_z, 'xyz_m': fit.x[:3].tolist(),
                 'rpy_rad': [0., 0., float(fit.x[3])], 'cost': float(fit.cost),
                 'evaluations': fit.nfev, 'success': bool(fit.success),
                 'surface_distances': _summary(residual(fit.x))}
        fits.append(entry)
        print(json.dumps({'fit': entry}), flush=True)
    best = min(fits, key=lambda entry: entry['cost'])
    configured = model_origin_matrix(config.get('model_origin_xyz', [0., 0., 0.]),
                                     config.get('model_origin_rpy', [0., 0., 0.]))
    candidates = {'uncorrected': np.eye(4),
                  'configured': configured,
                  'estimated': model_origin_matrix(best['xyz_m'], best['rpy_rad'])}
    rows = []
    for sample in samples:
        points = sample['points']
        near = np.linalg.norm(points[:, :2] - sample['matrix'][:2, 3], axis=1) < args.radius
        region = near & (points[:, 2] > args.min_height)
        high = near & (points[:, 2] > 2.7)
        for name, transform in candidates.items():
            row = {'frame_index': sample['index'], 'stamp_ns': sample['stamp_ns'],
                   'candidate': name, 'cloud_frame': sample['frame'],
                   'surface_distances': _summary(_distances(
                       points[region], sample, transform, collisions)), 'padding_checks': []}
            for padding in (0.15, 0.25, 0.30):
                model.padding = padding
                mask = model.mask(points, sample['matrix'] @ transform, sample['joints'])
                row['padding_checks'].append({
                    'padding_m': padding, 'removed': int(mask.sum()),
                    'central_high_candidates': int(high.sum()),
                    'central_high_retained': int(np.count_nonzero(high & ~mask)),
                    'removed_z_below_03m': int(np.count_nonzero(mask & (points[:, 2] < 0.3))),
                    'removed_z_below_08m': int(np.count_nonzero(
                        mask & (points[:, 2] < 0.8)))})
            rows.append(row)
    report = {
        'bag': str(bag_path), 'training_frame': training['index'],
        'method': 'Point-to-triangle body/crawler distances; bounded xyz/yaw soft_l1 fit',
        'region': {'radius_xy_m': args.radius, 'minimum_cloud_z_m': args.min_height},
        'fits': fits, 'best_estimated_origin_xyz': best['xyz_m'],
        'best_estimated_origin_rpy': best['rpy_rad'], 'validation': rows,
        'limitations': [
            'The region and search bounds are provisional choices for the supplied recording.',
            'High and low height bands are geometric proxies, not labelled robot/ground truth.',
            'Only visible body/crawler are fitted; occlusion or mesh mismatch may leave returns.',
            'This report does not alter GNSS measurements, parameters, or robot geometry.']}
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + '\n')
    if args.preview:
        _preview(args.preview, samples, model, configured,
                 config.get('self_filter_padding', 0.3))
    print(json.dumps({'report': str(output), 'best_xyz': best['xyz_m'],
                      'best_rpy': best['rpy_rad']}), flush=True)


def main():
    """Expose explicit bag-specific calibration region and search bounds."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', default='', help='Bag directory; default uses rosbag/ recording.')
    parser.add_argument('--params', default='')
    parser.add_argument('--output', default='/tmp/pointcloud_model_calibration.json')
    parser.add_argument('--preview', default='', help='Optional configured-origin comparison PNG.')
    parser.add_argument('--radius', type=float, default=3.0)
    parser.add_argument('--min-height', type=float, default=0.8)
    parser.add_argument('--initial-z', type=float, nargs='+', default=[-3.7, -3.4, -3.0, -2.5])
    parser.add_argument('--z-range', type=float, nargs=2, default=[-4.5, -1.5])
    args = parser.parse_args()
    if args.radius <= 0 or not np.isfinite(args.radius) or not np.isfinite(args.min_height):
        parser.error('radius must be finite and > 0; min-height must be finite')
    calibrate(args)


if __name__ == '__main__':
    main()
