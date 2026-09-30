import math
import pytest
from aps_ndt_localization.prediction import predict, rpy_quaternion, seed_times


def test_constant_velocity_and_yaw():
    first = (0., (0., 0., 0.), rpy_quaternion(0., 0., 0.))
    second = (1., (1., 2., 0.), rpy_quaternion(0., 0., .2))
    xyz, quat = predict(first, second, 1.5, 1.)
    assert xyz == pytest.approx((1.5, 3., 0.))
    assert quat == pytest.approx(rpy_quaternion(0., 0., .3))


def test_prediction_is_capped_and_first_pose_holds():
    initial = (0., (1., 2., 3.), (0., 0., 0., 1.))
    second = (1., (2., 2., 3.), (0., 0., 0., 1.))
    assert predict(None, initial, 99., .5) == (initial[1], initial[2])
    assert predict(initial, second, 99., .5)[0] == pytest.approx((2.5, 2., 3.))


def test_quaternion_sign_is_not_rotation():
    first = (0., (0., 0., 0.), (0., 0., 0., 1.))
    second = (1., (0., 0., 0.), (0., 0., 0., -1.))
    _, quat = predict(first, second, 1.5, 1.)
    assert abs(quat[3]) == pytest.approx(1.)


def test_seed_times_never_reverse_at_submillisecond_scan_intervals():
    assert seed_times(20_000_000, 19_500_000) == (19_500_001, 20_000_000)
    assert seed_times(20_000_001, 20_000_000) == (20_000_001,)
