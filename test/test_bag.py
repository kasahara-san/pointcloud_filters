"""Check source and installed recording resolution without bundling bag data."""

import pytest

from pointcloud_filters.bag import DEFAULT_BAG_NAME, resolve_bag_path


def _recording(root):
    path = root / 'rosbag' / DEFAULT_BAG_NAME
    path.mkdir(parents=True)
    (path / 'metadata.yaml').write_text('rosbag2_bagfile_information: {}\n')
    return path


def test_source_checkout_and_symlinked_install(tmp_path):
    """Resolve through module realpaths even when the launch is symlink-installed."""
    root = tmp_path / 'source'
    recording = _recording(root)
    module = root / 'pointcloud_filters' / 'bag.py'
    module.parent.mkdir()
    module.touch()
    installed = tmp_path / 'install' / 'bag.py'
    installed.parent.mkdir()
    installed.symlink_to(module)
    assert resolve_bag_path(anchors=[module]) == str(recording)
    assert resolve_bag_path(anchors=[installed]) == str(recording)


@pytest.mark.parametrize('install_layout', [
    'install/pointcloud_filters/lib/python3.10/site-packages/pointcloud_filters',
    'install/lib/python3.10/site-packages/pointcloud_filters',
])
def test_copied_colcon_install(tmp_path, install_layout):
    """Find source data from both isolated and merged installed Python modules."""
    recording = _recording(tmp_path / 'src' / 'pointcloud_filters')
    module = tmp_path / install_layout / 'bag.py'
    module.parent.mkdir(parents=True)
    module.touch()
    assert resolve_bag_path(anchors=[module]) == str(recording)


def test_explicit_directory_and_file(tmp_path, monkeypatch):
    """Honor a relative or absolute recording override before default discovery."""
    directory = tmp_path / 'custom'
    directory.mkdir()
    (directory / 'metadata.yaml').touch()
    standalone = tmp_path / 'standalone.db3'
    standalone.touch()
    monkeypatch.chdir(tmp_path)
    assert resolve_bag_path('custom') == str(directory)
    assert resolve_bag_path(str(standalone)) == str(standalone)


def test_missing_recording_has_actionable_error(tmp_path):
    """Missing data must fail before spawning a player with an invalid path."""
    with pytest.raises(ValueError, match='bag_path:='):
        resolve_bag_path(anchors=[tmp_path])
    with pytest.raises(ValueError, match='Invalid bag_path'):
        resolve_bag_path(str(tmp_path / 'missing'))
