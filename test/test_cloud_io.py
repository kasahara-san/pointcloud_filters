"""Check PointCloud2 interoperability without dropping binary attributes."""

from copy import deepcopy

import numpy as np
import pytest
from sensor_msgs.msg import PointCloud2, PointField

from pointcloud_filters.cloud_io import read_cloud, select_cloud


@pytest.mark.parametrize('bigendian', [False, True])
def test_organized_padding_endianness_and_extra_attributes(bigendian):
    """Read padded rows and preserve complete original point records."""
    cloud = PointCloud2()
    cloud.header.frame_id = 'map'
    cloud.header.stamp.sec = 123
    cloud.height, cloud.width = 2, 2
    cloud.point_step, cloud.row_step = 20, 48
    cloud.is_bigendian = bigendian
    cloud.is_dense = False
    cloud.fields = [
        PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
        for name, offset in [('x', 0), ('y', 4), ('z', 8)]
    ] + [PointField(name='intensity', offset=16,
                    datatype=PointField.UINT16, count=1)]
    data = bytearray([0xa5] * (cloud.row_step * cloud.height))
    order = '>' if bigendian else '<'
    dtype = np.dtype({'names': ['x', 'y', 'z', 'intensity'],
                      'formats': [order + 'f4'] * 3 + [order + 'u2'],
                      'offsets': [0, 4, 8, 16], 'itemsize': cloud.point_step})
    records = np.ndarray((2, 2), dtype=dtype, buffer=data, strides=(48, 20))
    records['x'] = [[1, 2], [3, 4]]
    records['y'] = 5
    records['z'] = 6
    records['intensity'] = [[11, 12], [13, 14]]
    cloud.data = bytes(data)
    original = deepcopy(cloud)
    xyz, raw = read_cloud(cloud)
    np.testing.assert_equal(xyz, [[1, 5, 6], [2, 5, 6], [3, 5, 6], [4, 5, 6]])
    result = select_cloud(cloud, raw, np.array([1, 2]))
    assert result.height == 1 and result.width == 2
    assert result.row_step == 40
    assert result.header == cloud.header
    assert result.fields == cloud.fields
    assert result.is_bigendian == bigendian
    assert bytes(result.data) == bytes(data[20:40] + data[48:68])
    assert cloud == original
    empty = select_cloud(cloud, raw, np.empty(0, dtype=np.intp))
    assert empty.width == empty.row_step == len(empty.data) == 0
    assert empty.header == cloud.header


def test_reject_malformed_cloud():
    """Reject an invalid binary layout before attempting spatial processing."""
    with pytest.raises(ValueError, match='point_step'):
        read_cloud(PointCloud2())
