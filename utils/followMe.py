import numpy as np
def generate_spiral_waypoints(tx0, ty0, tz0, n=5, stop_dist=0.5, radius=0.25, turns=0.75):
    wps = []
    for i in range(1, n + 1):
        s = i / float(n)

        r = radius * (1 - s)
        theta = 2.0 * np.pi * turns * s

        cx = (1 - s) * tx0
        cy = (1 - s) * ty0

        x = cx + r * np.cos(theta)
        y = cy + r * np.sin(theta)
        z = (1 - s) * tz0 + s * stop_dist

        wps.append(np.array([x, y, z], dtype=np.float32))
    return wps

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def rc_to_reach_tvec(txnow, tynow, tznow, waypoint, k_xy=120.0, k_z=140.0, k_yaw=80.0):
    """
    Maps (tvec error) -> Tello rc commands.
    Units: tvec in meters. rc outputs in [-100,100].
    """
    xerr = txnow - waypoint[0]
    yerr = tynow - waypoint[1]
    zerr = tznow - waypoint[2]

    # Move toward desired marker position in frame:
    lr  = clamp( k_xy * xerr, -60, 60)   
    ud  = clamp( k_xy * yerr, -60, 60)
    fb  = clamp( k_z  * zerr, -70, 70)
    return lr, fb, ud

def reached_waypoint(txnow, tynow, tznow, waypoint, tol_xyz=0.08):
    tvec_now = np.array([txnow, tynow, tznow])
    return float(np.linalg.norm(tvec_now - waypoint)) < tol_xyz