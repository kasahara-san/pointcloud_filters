"""Locate the bundled recording without copying multi-gigabyte data into install."""

from pathlib import Path


DEFAULT_BAG_NAME = 'rosbag2_2026_10_25-08_46_47'


def resolve_bag_path(bag_path='', anchors=None):
    """Resolve an explicit recording or find it in a source/colcon workspace."""
    if bag_path:
        path = Path(bag_path).expanduser().resolve()
        if path.is_file() or (path / 'metadata.yaml').is_file():
            return str(path)
        raise ValueError(f'Invalid bag_path: {path}; expected a rosbag file or directory')

    if anchors is None:
        anchors = (Path(__file__), Path.cwd())
    checked = set()
    for anchor in anchors:
        anchor = Path(anchor).resolve()
        if anchor.is_file():
            anchor = anchor.parent
        for ancestor in (anchor, *anchor.parents):
            # The first path covers source and --symlink-install. The second
            # covers copied installs in isolated and merged colcon workspaces.
            for source_root in (ancestor, ancestor / 'src' / 'pointcloud_filters'):
                path = source_root / 'rosbag' / DEFAULT_BAG_NAME
                if path in checked:
                    continue
                checked.add(path)
                if (path / 'metadata.yaml').is_file():
                    return str(path)
    raise ValueError(
        f'Cannot find rosbag/{DEFAULT_BAG_NAME}. Keep the recording in the source '
        'package or specify bag_path:=/absolute/path/to/recording. '
        'Use play_bag:=false for live topics.')
