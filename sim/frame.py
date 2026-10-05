"""Private robot frame (x forward into the building, y left) -> WGS-84 GPS."""
import math

R_EARTH = 6378137.0


def to_enu(x, y, psi_deg):
    """Rotate a private-frame point into local East/North metres around the anchor."""
    psi = math.radians(psi_deg)
    north = x * math.cos(psi) + y * math.sin(psi)
    east = x * math.sin(psi) - y * math.cos(psi)
    return east, north


def enu_to_gps(east, north, lat0, lon0):
    lat = lat0 + math.degrees(north / R_EARTH)
    lon = lon0 + math.degrees(east / (R_EARTH * math.cos(math.radians(lat0))))
    return lat, lon


def to_gps(x, y, lat0, lon0, psi_deg):
    return enu_to_gps(*to_enu(x, y, psi_deg), lat0, lon0)


def gps_to_enu(lat, lon, lat0, lon0):
    north = math.radians(lat - lat0) * R_EARTH
    east = math.radians(lon - lon0) * R_EARTH * math.cos(math.radians(lat0))
    return east, north


def _rebuild(path, beta):
    """Re-integrate the believed path with the heading error beta*odo removed from each increment."""
    out = [path[0][:2]]
    x, y = path[0][:2]
    for (x0, y0, _), (x1, y1, d1) in zip(path, path[1:]):
        a = -beta * d1
        dx, dy = x1 - x0, y1 - y0
        x += math.cos(a) * dx - math.sin(a) * dy
        y += math.sin(a) * dx + math.cos(a) * dy
        out.append((x, y))
    return out


def estimate_gyro_drift(path):
    """Loop closure: the Writer ends at the entrance (0, 0). Find the heading drift rate
    beta (rad per metre driven) whose removal brings the re-integrated path back to (0, 0)."""
    def err(beta):
        ex, ey = _rebuild(path, beta)[-1]
        return math.hypot(ex, ey)
    lo, hi = -0.02, 0.02
    best = min((lo + (hi - lo) * k / 400 for k in range(401)), key=err)
    step = (hi - lo) / 400
    for _ in range(40):                                  # golden-section-like refinement
        cands = [best - step, best, best + step]
        best = min(cands, key=err)
        step /= 2
    return best, err(0.0), err(best)


def correct_points(path, beta, points, odo):
    """Move each point by the correction found for the path at the same odometer reading."""
    fixed = _rebuild(path, beta)
    deltas = [(fx - x, fy - y) for (x, y, _), (fx, fy) in zip(path, fixed)]
    odos = [d for *_, d in path]
    out = []
    for (px, py), d in zip(points, odo):
        j = min(range(len(odos)), key=lambda k: abs(odos[k] - d))
        out.append((px + deltas[j][0], py + deltas[j][1]))
    return out, fixed
