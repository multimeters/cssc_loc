"""Validate and optionally reduce XYZ clouds without discarding their original fields."""
import numpy as np


def selected_points(xyz, voxel_size):
    indices = np.flatnonzero(np.isfinite(xyz).all(axis=1))
    if voxel_size > 0 and len(indices):
        cells = np.floor(xyz[indices] / voxel_size)
        _, first = np.unique(cells, axis=0, return_index=True)
        indices = indices[np.sort(first)]
    return indices


def filter_cloud(scan, voxel_size):
    from sensor_msgs.msg import PointCloud2
    fields = {field.name: field for field in scan.fields}
    if scan.width < 1 or scan.height < 1 or scan.point_step < 12:
        raise ValueError("empty or invalid point cloud dimensions")
    if scan.row_step < scan.width*scan.point_step:
        raise ValueError("row_step smaller than width*point_step")
    required = (scan.height-1)*scan.row_step+scan.width*scan.point_step
    if len(scan.data) < required:
        raise ValueError("point cloud byte buffer is truncated")
    for name in ("x", "y", "z"):
        field = fields.get(name)
        if field is None or field.datatype != 7 or field.count != 1:
            raise ValueError("NDT requires scalar float32 XYZ fields")
        if field.offset < 0 or field.offset+4 > scan.point_step:
            raise ValueError("XYZ field offset outside point_step")
    dtype = np.dtype({
        "names": ["x", "y", "z"],
        "formats": [">f4" if scan.is_bigendian else "<f4"]*3,
        "offsets": [fields[name].offset for name in ("x", "y", "z")],
        "itemsize": scan.point_step,
    })
    data = memoryview(scan.data)
    structured = np.ndarray((scan.height, scan.width), dtype=dtype, buffer=data,
                            strides=(scan.row_step, scan.point_step))
    xyz = np.stack([structured[name].ravel() for name in ("x", "y", "z")], axis=1)
    indices = selected_points(xyz, voxel_size)
    if not len(indices):
        raise ValueError("point cloud has no finite XYZ points")
    if len(indices) == scan.width*scan.height:
        return scan
    raw = np.ndarray((scan.height, scan.width, scan.point_step), dtype=np.uint8,
                     buffer=data, strides=(scan.row_step, scan.point_step, 1))
    compact = raw.reshape((-1, scan.point_step))[indices].tobytes()
    return PointCloud2(header=scan.header, height=1, width=len(indices), fields=scan.fields,
                       is_bigendian=scan.is_bigendian, point_step=scan.point_step,
                       row_step=len(indices)*scan.point_step, data=compact, is_dense=True)
