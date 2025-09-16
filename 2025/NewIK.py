# leg_ik_foot_constrained.py
# IK with:
# - Foot contact point as the goal
# - Foot link always vertical (down) with length Lf (100 mm default)
# - Tibia tilt limited to ±tibia_dev_deg relative to VERTICAL DOWN (−pi/2)
# - Joint limits on yaw (q1), hip pitch (q2), knee pitch (q3)
# - Hip yaw -> hip pitch offset vector o (default [52, 0, 0] mm)

import numpy as np

# ---------- helpers ----------
def clamp(x, lo, hi):
    return np.minimum(np.maximum(x, lo), hi)

def fk_ball_point(q1, q2, q3, o, L1, L2):
    c1, s1 = np.cos(q1), np.sin(q1)
    x_sag = np.array([c1, s1, 0.0])
    z_ax  = np.array([0.0, 0.0, 1.0])
    fem = np.cos(q2)*x_sag + np.sin(q2)*z_ax
    tib = np.cos(q2+q3)*x_sag + np.sin(q2+q3)*z_ax
    return o + L1*fem + L2*tib

def fk_foot_contact(q1, q2, q3, o, L1, L2, Lf):
    ball = fk_ball_point(q1, q2, q3, o, L1, L2)
    return ball + np.array([0.0, 0.0, -Lf])

def branch_name(q3):
    return "down" if q3 >= 0.0 else "up"

# ---------- main solver ----------
def leg_ik_with_foot_target(
    p_contact,
    o,
    L1, L2, Lf=100.0,
    tibia_dev_deg=20.0,
    lim1_deg=(-90.0, 90.0),
    lim2_deg=(-120.0, 120.0),
    lim3_deg=(-120.0, 120.0),
    samples=181,
    tol_mm=1e-3,
    prefer="down"
):
    p_contact = np.array(p_contact, float)
    o = np.array(o, float)

    p_ball = p_contact + np.array([0.0, 0.0, Lf])
    d = p_ball - o

    d2r = np.pi / 180.0
    lim1 = np.array(lim1_deg) * d2r
    lim2 = np.array(lim2_deg) * d2r
    lim3 = np.array(lim3_deg) * d2r

    q1_star = np.arctan2(d[1], d[0])
    q1 = clamp(q1_star, lim1[0], lim1[1])

    c, s = np.cos(-q1), np.sin(-q1)
    Rz = np.array([[c, -s, 0],
                   [s,  c, 0],
                   [0,  0, 1]])
    dp = Rz @ d
    r, z = float(np.hypot(dp[0], dp[1])), float(dp[2])

    dev = float(tibia_dev_deg) * d2r
    theta_center = -np.pi/2
    theta_lo = theta_center - dev
    theta_hi = theta_center + dev
    thetas = np.linspace(theta_lo, theta_hi, int(samples))

    best_ok = None
    best_any = None

    for theta in thetas:
        kd_r = r - L2 * np.cos(theta)
        kd_z = z - L2 * np.sin(theta)

        if kd_r == 0.0 and kd_z == 0.0:
            kd_r = 1e-12
        q2_star = np.arctan2(kd_z, kd_r)
        q3_star = theta - q2_star

        q2 = clamp(q2_star, lim2[0], lim2[1])
        q3 = clamp(q3_star, lim3[0], lim3[1])

        theta_actual = q2 + q3
        dev_actual = abs(theta_actual + np.pi/2)
        tibia_ok = dev_actual <= (dev + 1e-9)

        p_hat = fk_foot_contact(q1, q2, q3, o, L1, L2, Lf)
        err = float(np.linalg.norm(p_hat - p_contact))
        cand = dict(
            q1=q1, q2=q2, q3=q3,
            err_mm=err,
            tibia_ok=bool(tibia_ok),
            tibia_tilt_deg=float(np.degrees(theta_actual + np.pi/2)),
            branch=branch_name(q3)
        )

        if (best_any is None) or (cand["err_mm"] < best_any["err_mm"] - 1e-12):
            best_any = cand

        if cand["tibia_ok"]:
            if (best_ok is None) or (cand["err_mm"] < best_ok["err_mm"] - 1e-12):
                best_ok = cand
            elif abs(cand["err_mm"] - best_ok["err_mm"]) <= 1e-12:
                if prefer == "down" and cand["branch"] == "down" and best_ok["branch"] != "down":
                    best_ok = cand
                if prefer == "up" and cand["branch"] == "up" and best_ok["branch"] != "up":
                    best_ok = cand

    best = best_ok if best_ok is not None else best_any
    qdeg = tuple(np.degrees([best["q1"], best["q2"], best["q3"]]))
    best["qdeg"] = qdeg
    best["feasible"] = (best["tibia_ok"] and best["err_mm"] <= tol_mm)
    return best

# ---------- demo ----------
if __name__ == "__main__":
    # Geometry (mm)
    L1, L2 = 107.0, 107.5
    o = np.array([52.0, 0.0, 0.0])
    Lf = 100.0

    # Limits (deg)
    lim1 = (-90, 90)
    lim2 = (-120, 120)
    lim3 = (-120, 120)

    # === NEW: user input for target ===
    line = input("Enter desired foot x y z (mm), e.g. '45 45 -100': ").strip()
    try:
        parts = line.split()
        if len(parts) >= 3:
            x, y, z = map(float, parts[:3])
            p_contact = np.array([x, y, z])
        else:
            raise ValueError
    except Exception:
        print("Invalid input, using default [159, 0, -207.5]")
        p_contact = np.array([159, 0, -207.5])

    sol = leg_ik_with_foot_target(
        p_contact, o, L1, L2, Lf=Lf,
        tibia_dev_deg=20.0,
        lim1_deg=lim1, lim2_deg=lim2, lim3_deg=lim3,
        samples=181, tol_mm=1e-3, prefer="down"
    )

    print("\nTarget foot contact (mm):", p_contact)
    print("q1,q2,q3 (deg):", np.round(sol["qdeg"], 3))
    print("Branch:", sol["branch"])
    print("Tibia tilt vs vertical DOWN (deg):", round(sol["tibia_tilt_deg"], 3))
    print("Error (mm):", round(sol["err_mm"], 4))
    print("Tibia tilt within ±20°:", sol["tibia_ok"])
    print("Feasible (tilt ok & error ≤ tol):", sol["feasible"])
