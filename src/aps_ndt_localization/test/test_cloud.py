import numpy as np
from aps_ndt_localization.cloud import selected_points


def test_filters_nonfinite_and_preserves_order():
    xyz = np.array([[1., 2., 3.], [np.nan, 2., 3.], [4., 5., 6.], [0., np.inf, 0.]])
    assert selected_points(xyz, 0.).tolist() == [0, 2]


def test_voxel_keeps_first_point_and_preserves_original_fields_via_indices():
    xyz = np.array([[0., 0., 0.], [.1, .1, .1], [1., 2., 3.], [-.1, 0., 0.]])
    assert selected_points(xyz, .3).tolist() == [0, 2, 3]
