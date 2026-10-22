"""Read and select PointCloud2 records while preserving their binary layout."""

from copy import deepcopy

import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import dtype_from_fields


def read_cloud(message):
    """Return XYZ and raw records, respecting endianness and organized row padding."""
    if message.point_step <= 0:
        raise ValueError('point_step must be positive')
    if message.row_step < message.width * message.point_step:
        raise ValueError('row_step is smaller than the point data in a row')
    if len(message.data) < message.height * message.row_step:
        raise ValueError('PointCloud2 data is shorter than its declared dimensions')
    dtype = dtype_from_fields(message.fields, point_step=message.point_step)
    dtype = dtype.newbyteorder('>' if message.is_bigendian else '<')
    for name in ('x', 'y', 'z'):
        if name not in dtype.names or dtype[name].kind != 'f':
            raise ValueError(f'PointCloud2 requires a scalar floating-point {name} field')
    records = np.ndarray(
        shape=(message.height, message.width), dtype=dtype, buffer=message.data,
        strides=(message.row_step, message.point_step))
    xyz = np.column_stack([records[name].ravel() for name in ('x', 'y', 'z')])
    # Byte views preserve additional fields and even padding inside each record.
    raw = np.ndarray(
        shape=(message.height, message.width, message.point_step), dtype=np.uint8,
        buffer=message.data, strides=(message.row_step, message.point_step, 1))
    return xyz, raw.reshape(-1, message.point_step)


def select_cloud(message, raw_records, indices):
    """Create an unorganized output cloud with the original header and fields."""
    result = PointCloud2()
    result.header = deepcopy(message.header)
    result.height = 1
    result.width = len(indices)
    result.fields = deepcopy(message.fields)
    result.is_bigendian = message.is_bigendian
    result.point_step = message.point_step
    result.row_step = result.width * result.point_step
    result.data = raw_records[indices].tobytes()
    # Only XYZ finiteness is guaranteed; retain a conservative attribute flag.
    result.is_dense = message.is_dense
    return result
