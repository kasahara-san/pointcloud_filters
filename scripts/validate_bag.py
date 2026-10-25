#!/usr/bin/env python3
"""
Validate every recorded cloud using the production geometry and filter code.

This checks geometric consistency with the supplied model. It cannot establish
that every physical robot return matches that model or measure false removals
without labelled ground truth.
"""

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import time

import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer
import yaml

from pointcloud_filters.bag import resolve_bag_path
from pointcloud_filters.cloud_io import read_cloud, select_cloud
from pointcloud_filters.filtering import FilterParameters, filter_indices
from pointcloud_filters.meshing import MeshConverter
from pointcloud_filters.position import model_origin_matrix, pose_matrix_in_cloud_frame
from pointcloud_filters.self_filter import SelfFilter
from pointcloud_filters.state_history import stamp_ns, StateHistory
from pointcloud_filters.visualization import mesh_marker


def _messages(bag_path, selected):
    path = Path(bag_path)
    metadata_path = path / 'metadata.yaml'
    storage_id = 'mcap' if path.suffix == '.mcap' else 'sqlite3'
    if metadata_path.is_file():
        metadata = yaml.safe_load(metadata_path.read_text())
        storage_id = metadata['rosbag2_bagfile_information']['storage_identifier']
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=storage_id),
                rosbag2_py.ConverterOptions('', ''))
    available = {item.name: item.type for item in reader.get_all_topics_and_types()}
    types = {name: get_message(available[name]) for name in selected if name in available}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
    while reader.has_next():
        name, data, recorded_ns = reader.read_next()
        yield name, deserialize_message(data, types[name]), recorded_ns


def _check(condition, message):
    if not condition:
        raise AssertionError(message)


def _nearest_difference(times, timestamp):
    values = np.asarray(times, dtype=np.int64)
    return float(np.min(np.abs(values - timestamp)) * 1e-9)


def _preview(path, xyz, model, matrix, model_matrix, joints):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    transforms = model.link_transforms(joints)
    mask = model.mask(xyz, model_matrix, joints)
    figure, axes = plt.subplots(1, 3, figsize=(16, 6))
    for axis, (horizontal, vertical) in zip(axes, [(0, 1), (0, 2), (1, 2)]):
        axis.scatter(xyz[:, horizontal], xyz[:, vertical], c='gray', s=2,
                     label='Recorded cloud')
        axis.scatter(xyz[mask, horizontal], xyz[mask, vertical], c='red', s=20,
                     label='Removed by model')
        axis.scatter(matrix[horizontal, 3], matrix[vertical, 3], c='black',
                     marker='+', s=80, label='GNSS base origin')
        for collision in model.collisions:
            transform = model_matrix @ transforms[collision.link] @ collision.origin
            vertices = collision.vertices @ transform[:3, :3].T + transform[:3, 3]
            axis.scatter(vertices[:, horizontal], vertices[:, vertical],
                         s=0.4, alpha=0.3, label=collision.link + ' original')
            if model.scale > 1:
                center = (collision.lower + collision.upper) / 2
                scaled = center + (collision.vertices - center) * model.scale
                scaled = scaled @ transform[:3, :3].T + transform[:3, 3]
                axis.scatter(scaled[:, horizontal], scaled[:, vertical], s=0.4, alpha=0.3,
                             label=f'{collision.link} x{model.scale:g}')
        axis.set_aspect('equal')
        axis.grid()
        axis.set_xlabel('XYZ'[horizontal] + ' [m]')
        axis.set_ylabel('XYZ'[vertical] + ' [m]')
    axes[0].legend(fontsize=6, markerscale=2)
    figure.suptitle('First recorded cloud and collision model; configured CAD origin')
    figure.tight_layout()
    figure.savefig(path, dpi=160)
    plt.close(figure)


def validate(args):
    """Process all clouds, then reconstruct all five mesh levels on sample frames."""
    started = time.monotonic()
    root = Path(__file__).resolve().parents[1]
    config_path = Path(args.params) if args.params else root / 'config/pointcloud_filter.yaml'
    config = yaml.safe_load(config_path.read_text())['pointcloud_filter']['ros__parameters']
    parameters = FilterParameters(**{
        name: config.get(name, value) for name, value in asdict(FilterParameters()).items()})
    bag_path = resolve_bag_path(args.bag)
    robot_path = config.get('robot_description_path') or root / 'robot/zx200.urdf'
    model = SelfFilter(robot_path, padding=config.get('self_filter_padding', 0.15),
                       scale=config.get('self_filter_scale', 1.0))
    baseline_model = (SelfFilter(robot_path, padding=model.padding)
                      if model.scale > 1 else model)
    base_from_model = model_origin_matrix(
        config.get('model_origin_xyz', [0., 0., 0.]),
        config.get('model_origin_rpy', [0., 0., 0.]))
    pose_topic = config.get('pose_topic', '/zx200/global_pose')
    joint_topic = config.get('joint_states_topic', '/zx200/joint_states')
    cloud_topic = config.get('input_topic', '/merged_pointcloud')
    pose_tolerance = config.get('max_pose_time_diff_sec', 0.5)
    joint_tolerance = config.get('max_joint_time_diff_sec', 0.5)
    # Offline validation retains the complete recording, using the same nearest
    # measurement selection as the node without the live memory limit.
    history = StateHistory(duration_sec=1e9)
    tf_buffer = Buffer()
    joint_times = {name: [] for name in model.required_joints}
    pose_times, cloud_times = [], []
    topic_counts, static_edges = {}, []
    for name, message, _ in _messages(
            bag_path, [pose_topic, joint_topic, cloud_topic, '/tf_static']):
        topic_counts[name] = topic_counts.get(name, 0) + 1
        if name == pose_topic:
            history.add_pose(message)
            pose_times.append(stamp_ns(message.header.stamp))
        elif name == joint_topic:
            history.add_joints(message)
            for joint in joint_times:
                if joint in message.name:
                    joint_times[joint].append(stamp_ns(message.header.stamp))
        elif name == '/tf_static':
            for transform in message.transforms:
                tf_buffer.set_transform_static(transform, 'recording_validation')
                static_edges.append({
                    'parent': transform.header.frame_id,
                    'child': transform.child_frame_id,
                    'translation': [getattr(transform.transform.translation, axis)
                                    for axis in ('x', 'y', 'z')],
                    'quaternion_xyzw': [getattr(transform.transform.rotation, axis)
                                        for axis in ('x', 'y', 'z', 'w')]})
        else:
            cloud_times.append(stamp_ns(message.header.stamp))
    _check(bool(cloud_times), f'No clouds on {cloud_topic}')
    _check(bool(pose_times), f'No poses on {pose_topic}')
    _check(all(joint_times.values()), 'Missing required joint measurements')
    samples = {0, len(cloud_times) // 2, len(cloud_times) - 1}
    retained_samples, frames = [], []
    for index, (_, cloud, _) in enumerate(_messages(bag_path, [cloud_topic])):
        frame_started = time.monotonic()
        timestamp = stamp_ns(cloud.header.stamp)
        pose = history.pose_at(timestamp, pose_tolerance)
        joints = history.joints_at(timestamp, model.required_joints, joint_tolerance)
        matrix = pose_matrix_in_cloud_frame(
            pose, cloud.header, tf_buffer, pose_tolerance, config.get('pose_frame', 'world'))
        model_matrix = matrix @ base_from_model
        xyz, records = read_cloud(cloud)
        mask = np.zeros(len(xyz), dtype=bool)
        candidates = np.flatnonzero(
            np.isfinite(xyz).all(axis=1)
            & (np.hypot(xyz[:, 0] - matrix[0, 3], xyz[:, 1] - matrix[1, 3])
               <= parameters.radius_m))
        mask[candidates] = model.mask(xyz[candidates], model_matrix, joints)
        baseline_mask = baseline_model.mask(xyz[candidates], model_matrix, joints)
        _check(mask[candidates][baseline_mask].all(),
               f'Frame {index}: model growth lost previously excluded points')
        indices, counts = filter_indices(xyz, matrix[:2, 3], parameters, mask)
        result = select_cloud(cloud, records, indices)
        output_xyz, output_records = read_cloud(result)
        _check(result.header == cloud.header, f'Frame {index}: changed Header')
        _check(result.fields == cloud.fields, f'Frame {index}: changed point fields')
        _check(result.point_step == cloud.point_step, f'Frame {index}: changed point_step')
        _check(np.array_equal(output_records, records[indices]),
               f'Frame {index}: point attributes or padding changed')
        _check(np.array_equal(output_xyz, xyz[indices]), f'Frame {index}: changed XYZ')
        _check(np.isfinite(output_xyz).all(), f'Frame {index}: nonfinite output')
        _check(not model.mask(output_xyz, model_matrix, joints).any(),
               f'Frame {index}: robot-model points remain in output')
        _check(np.all(np.hypot(output_xyz[:, 0] - matrix[0, 3],
                               output_xyz[:, 1] - matrix[1, 3]) <= parameters.radius_m),
               f'Frame {index}: output outside radius')
        counts.update(
            frame=index, stamp_ns=timestamp, center_xyz=matrix[:3, 3].tolist(),
            unscaled_self_removed=int(np.count_nonzero(baseline_mask)),
            self_removed_by_growth=int(np.count_nonzero(mask) - np.count_nonzero(baseline_mask)),
            pose_time_difference_sec=abs(timestamp - stamp_ns(pose.header.stamp)) * 1e-9,
            max_joint_time_difference_sec=max(
                _nearest_difference(times, timestamp) for times in joint_times.values()),
            self_removed_below_0_8m=int(np.count_nonzero(mask & (xyz[:, 2] < 0.8))),
            self_removed_below_0m=int(np.count_nonzero(mask & (xyz[:, 2] < 0.0))),
            processing_sec=time.monotonic() - frame_started,
            residual_model_points=0)
        frames.append(counts)
        if index == 0 and args.preview:
            _preview(args.preview, xyz[candidates], model, matrix, model_matrix, joints)
        if index in samples:
            retained_samples.append((index, output_xyz, model_matrix, joints, cloud.header))
        if index % 25 == 0 or index == len(cloud_times) - 1:
            print(f'frame={index}/{len(cloud_times) - 1} input={counts["input"]} '
                  f'self_removed={counts["self_removed"]} output={counts["output"]}',
                  flush=True)
    mesh_results = []
    if not args.skip_mesh:
        for resolution in range(1, 6):
            converter = MeshConverter(resolution)
            for index, points, matrix, joints, header in retained_samples:
                mesh_started = time.monotonic()

                def excluded(query):
                    return model.mask(query, matrix, joints)

                mesh, counts = converter.convert(
                    points, excluded=excluded,
                    exclusion_step=max(0.01, min(0.075, model.padding / 2)))
                vertices = np.array([[v.x, v.y, v.z] for v in mesh.vertices]).reshape(-1, 3)
                triangles = np.array([list(t.vertex_indices) for t in mesh.triangles],
                                     dtype=np.int64).reshape(-1, 3)
                _check(len(vertices) > 0 and len(triangles) > 0,
                       f'Frame {index} resolution {resolution}: empty mesh')
                _check(np.isfinite(vertices).all(), 'Mesh contains nonfinite vertices')
                _check(triangles.min() >= 0 and triangles.max() < len(vertices),
                       'Mesh contains invalid triangle indices')
                _check(not excluded(vertices).any(), 'Mesh vertices intersect robot model')
                corners = vertices[triangles]
                centroids = corners.mean(axis=1)
                midpoints = (corners + np.roll(corners, 1, axis=1)) / 2
                _check(not excluded(centroids).any(),
                       'Mesh triangle centroids intersect robot model')
                _check(not excluded(midpoints.reshape(-1, 3)).any(),
                       'Mesh edge midpoints intersect robot model')
                marker = mesh_marker(mesh, header)
                _check(marker.header == header, 'Mesh marker changed Header')
                _check(len(marker.points) == 3 * len(triangles), 'Invalid mesh marker')
                counts.update(frame=index, resolution=resolution,
                              processing_sec=time.monotonic() - mesh_started)
                mesh_results.append(counts)
                print(f'mesh frame={index} resolution={resolution} '
                      f'vertices={len(vertices)} triangles={len(triangles)}', flush=True)
    return {
        'status': 'passed', 'bag_path': bag_path, 'parameters_file': str(config_path),
        'robot_description': str(robot_path), 'filter_parameters': asdict(parameters),
        'self_filter_padding': model.padding, 'self_filter_scale': model.scale,
        'required_joints': list(model.required_joints),
        'model_origin_xyz': base_from_model[:3, 3].tolist(),
        'base_from_model': base_from_model.tolist(),
        'topic_counts': topic_counts, 'static_transforms': static_edges,
        'frames_checked': len(frames),
        'total_self_removed': sum(frame['self_removed'] for frame in frames),
        'total_unscaled_self_removed': sum(f['unscaled_self_removed'] for f in frames),
        'total_additional_growth_removals': sum(f['self_removed_by_growth'] for f in frames),
        'low_z_removal_proxy': {
            'coordinate': 'PointCloud2 frame Z, in metres',
            'threshold_m': 0.8,
            'total_below_threshold': sum(f['self_removed_below_0_8m'] for f in frames),
            'frames_with_removals_below_threshold': [
                f['frame'] for f in frames if f['self_removed_below_0_8m']],
            'total_below_zero': sum(f['self_removed_below_0m'] for f in frames),
            'interpretation': 'A low terrain removal proxy for this recording; '
                              'not a labelled false-removal measurement.'},
        'output_points_min_max': [min(f['output'] for f in frames),
                                  max(f['output'] for f in frames)],
        'max_pose_time_difference_sec': max(f['pose_time_difference_sec'] for f in frames),
        'max_joint_time_difference_sec': max(f['max_joint_time_difference_sec'] for f in frames),
        'filter_runtime_sec_median_max': [float(np.median([f['processing_sec'] for f in frames])),
                                          max(f['processing_sec'] for f in frames)],
        'residual_model_points': sum(f['residual_model_points'] for f in frames),
        'mesh_checks': mesh_results, 'frames': frames,
        'elapsed_sec': time.monotonic() - started,
        'validation_scope': 'Consistency with the specified collision model; '
                            'physical robot recall and false-removal rates need labelled data.'}


def main():
    """Write a reproducible JSON report; assertion failures return a nonzero status."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', default='', help='Default: bundled October 25 recording.')
    parser.add_argument('--params', default='', help='Default: package configuration YAML.')
    parser.add_argument('--report', default='/tmp/pointcloud_filters_validation.json')
    parser.add_argument('--preview', default='',
                        help='Optional PNG overlay of the first cloud and collision model; '
                             'requires matplotlib.')
    parser.add_argument('--skip-mesh', action='store_true',
                        help='Only validate all PointCloud2 frames.')
    args = parser.parse_args()
    report = validate(args)
    Path(args.report).write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({key: value for key, value in report.items()
                      if key not in ('frames', 'static_transforms', 'mesh_checks')}, indent=2))
    print(f'Report: {args.report}')


if __name__ == '__main__':
    main()
