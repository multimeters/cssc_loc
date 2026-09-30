"""Rigid transforms in xyzw quaternion convention; no ROS dependency."""
import math


def quaternion_rpy(roll, pitch, yaw):
    sr, cr = math.sin(roll / 2), math.cos(roll / 2)
    sp, cp = math.sin(pitch / 2), math.cos(pitch / 2)
    sy, cy = math.sin(yaw / 2), math.cos(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def quaternion_multiply(left, right):
    x, y, z, w = left
    a, b, c, d = right
    return (w * a + x * d + y * c - z * b,
            w * b - x * c + y * d + z * a,
            w * c + x * b - y * a + z * d,
            w * d - x * a - y * b - z * c)


def rotate(quaternion, vector):
    inverse = tuple(-v for v in quaternion[:3]) + (quaternion[3],)
    return quaternion_multiply(quaternion_multiply(quaternion, (*vector, 0.)), inverse)[:3]


def body_pose_to_base(body_xyz, body_rpy, mounting_xyz, mounting_rpy):
    """T_map_base = T_map_body * inverse(T_base_body)."""
    values = (*body_xyz, *body_rpy, *mounting_xyz, *mounting_rpy)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Initial pose and mounting transform must be finite')
    q_map_body = quaternion_rpy(*body_rpy)
    q_base_body = quaternion_rpy(*mounting_rpy)
    q_body_base = tuple(-v for v in q_base_body[:3]) + (q_base_body[3],)
    q_map_base = quaternion_multiply(q_map_body, q_body_base)
    offset = rotate(q_map_base, mounting_xyz)
    return tuple(body_xyz[i] - offset[i] for i in range(3)), q_map_base


def angular_velocity_to_base(rates, covariance, q_base_imu, variance_floor):
    """Rotate an IMU vector and its full covariance using target<-source R.

    Unknown diagonal variances receive the configured floor. Correlations are
    retained and symmetrized. An invalid covariance receives the conservative
    diagonal shift needed for strict diagonal dominance rather than silently
    discarding its correlations.
    """
    if (len(rates) != 3 or len(covariance) != 9 or len(q_base_imu) != 4
            or variance_floor <= 0 or not math.isfinite(variance_floor)
            or not all(math.isfinite(value) for value in (*rates, *q_base_imu))):
        raise ValueError('Invalid IMU vector, transform or covariance dimensions')
    norm = sum(value * value for value in q_base_imu)
    if abs(norm - 1.) > 1e-6:
        raise ValueError('IMU rotation quaternion must have unit norm')
    matrix = [[0.] * 3 for _ in range(3)]
    for row in range(3):
        value = covariance[4 * row]
        matrix[row][row] = max(value, variance_floor) if math.isfinite(value) else variance_floor
        for column in range(row):
            values = (covariance[3 * row + column], covariance[3 * column + row])
            element = sum(values) / 2 if all(math.isfinite(value) for value in values) else 0.
            matrix[row][column] = matrix[column][row] = element
    # Test positive definiteness by Cholesky before changing any valid covariance.
    cholesky = [[0.] * 3 for _ in range(3)]
    positive = True
    for row in range(3):
        for column in range(row + 1):
            value = matrix[row][column] - sum(cholesky[row][k] * cholesky[column][k]
                                              for k in range(column))
            if row == column:
                if value <= 0:
                    positive = False
                    break
                cholesky[row][column] = math.sqrt(value)
            else:
                cholesky[row][column] = value / cholesky[column][column]
        if not positive:
            break
    if not positive:
        lower_bound = min(matrix[row][row] - sum(abs(matrix[row][column])
                                                 for column in range(3) if row != column)
                          for row in range(3))
        shift = max(0., variance_floor - lower_bound)
        for row in range(3):
            matrix[row][row] += shift
    columns = [rotate(q_base_imu, basis) for basis in ((1., 0., 0.), (0., 1., 0.), (0., 0., 1.))]
    rotation = [[columns[column][row] for column in range(3)] for row in range(3)]
    transformed = [sum(rotation[row][a] * matrix[a][b] * rotation[column][b]
                       for a in range(3) for b in range(3))
                   for row in range(3) for column in range(3)]
    return rotate(q_base_imu, rates), transformed


def fusion_mode(now, latest, sensor_timeout, ndt_timeout, prediction_timeout, future_tolerance):
    """Classify short outages without permanently latching an IMU-only failure."""
    if now <= 0:
        return 'WAITING', False
    gyro = latest.get('gyro')
    ndt = latest.get('ndt')
    fresh_gyro = gyro is not None and -future_tolerance <= now - gyro <= sensor_timeout
    fresh_ndt = ndt is not None and -future_tolerance <= now - ndt <= ndt_timeout
    if fresh_gyro and fresh_ndt:
        return 'FUSED', True
    if fresh_ndt:
        return 'NDT_ONLY', True
    if fresh_gyro:
        return 'GYRO_ONLY', True
    stamps = [stamp for stamp in (gyro, ndt) if stamp is not None and stamp <= now + future_tolerance]
    if stamps and now - max(stamps) <= prediction_timeout:
        return 'PREDICTING', True
    return 'STALE', False
