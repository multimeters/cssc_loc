"""Small pure-math constant-velocity prior, derived only from accepted NDT results."""
import math


def seed_times(scan_ns, previous_seed_ns):
    first = scan_ns-1_000_000
    if previous_seed_ns is not None:
        first = max(first, previous_seed_ns+1)
    return (first, scan_ns) if first < scan_ns else (scan_ns,)


def normalized(q):
    length = math.sqrt(sum(v*v for v in q))
    if not math.isfinite(length) or length < 1e-10:
        raise ValueError("Invalid quaternion")
    return tuple(v / length for v in q)


def multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw*bx+ax*bw+ay*bz-az*by,
            aw*by-ax*bz+ay*bw+az*bx,
            aw*bz+ax*by-ay*bx+az*bw,
            aw*bw-ax*bx-ay*by-az*bz)


def rpy_quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return normalized((sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
                       cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy))


def predict(previous, latest, stamp, max_extrapolation):
    """State is (seconds, xyz tuple, xyzw tuple); hold until two accepted poses exist."""
    if previous is None:
        return latest[1], latest[2]
    dt = latest[0]-previous[0]
    if dt <= 1e-6:
        return latest[1], latest[2]
    horizon = max(0.0, min(stamp-latest[0], max_extrapolation))
    ratio = horizon/dt
    xyz = tuple(b+(b-a)*ratio for a, b in zip(previous[1], latest[1]))
    qa, qb = normalized(previous[2]), normalized(latest[2])
    delta = normalized(multiply((-qa[0], -qa[1], -qa[2], qa[3]), qb))
    if delta[3] < 0:
        delta = tuple(-v for v in delta)
    half_angle = math.acos(max(-1.0, min(1.0, delta[3])))
    if half_angle < 1e-8:
        return xyz, qb
    scale = math.sin(half_angle*ratio)/math.sin(half_angle)
    future = (delta[0]*scale, delta[1]*scale, delta[2]*scale, math.cos(half_angle*ratio))
    return xyz, normalized(multiply(qb, future))
