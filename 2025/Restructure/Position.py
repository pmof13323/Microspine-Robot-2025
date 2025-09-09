import numpy as np
import time
from TransmitData import OpenRB

# --- helpers ---
def clamp(x, lo, hi): return np.minimum(np.maximum(x, lo), hi)

def Rz(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c,-s,0.0],
                     [s, c,0.0],
                     [0.0,0.0,1.0]])

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

def branch_name(q3): return "down" if q3 >= 0.0 else "up"

def leg_ik_with_foot_target(
    p_contact, o, L1, L2, Lf=100.0,
    tibia_dev_deg=20.0,
    lim1_deg=(-90.0, 90.0),
    lim2_deg=(-120.0, 120.0),
    lim3_deg=(-120.0, 120.0),
    samples=181, tol_mm=1e-3, prefer="down"
):
    p_contact = np.array(p_contact, float); o = np.array(o, float)
    p_ball = p_contact + np.array([0.0, 0.0, Lf])
    d = p_ball - o

    d2r = np.pi/180.0
    lim1 = np.array(lim1_deg)*d2r; lim2 = np.array(lim2_deg)*d2r; lim3 = np.array(lim3_deg)*d2r

    q1_star = np.arctan2(d[1], d[0]); q1 = clamp(q1_star, lim1[0], lim1[1])

    c, s = np.cos(-q1), np.sin(-q1)
    Rz_local = np.array([[c,-s,0],[s,c,0],[0,0,1]])
    dp = Rz_local @ d; r, z = float(np.hypot(dp[0], dp[1])), float(dp[2])

    dev = float(tibia_dev_deg)*d2r; theta_center = -np.pi/2
    theta_lo, theta_hi = theta_center - dev, theta_center + dev
    thetas = np.linspace(theta_lo, theta_hi, int(samples))

    best_ok = None; best_any = None
    for theta in thetas:
        kd_r = r - L2*np.cos(theta); kd_z = z - L2*np.sin(theta)
        if kd_r == 0.0 and kd_z == 0.0: kd_r = 1e-12
        q2_star = np.arctan2(kd_z, kd_r); q3_star = theta - q2_star
        q2 = clamp(q2_star, lim2[0], lim2[1]); q3 = clamp(q3_star, lim3[0], lim3[1])

        theta_actual = q2 + q3
        dev_actual = abs(theta_actual + np.pi/2)
        tibia_ok = dev_actual <= (dev + 1e-9)

        p_hat = fk_foot_contact(q1, q2, q3, o, L1, L2, Lf)
        err = float(np.linalg.norm(p_hat - p_contact))
        cand = dict(q1=q1,q2=q2,q3=q3, err_mm=err, tibia_ok=bool(tibia_ok),
                    tibia_tilt_deg=float(np.degrees(theta_actual + np.pi/2)),
                    branch=branch_name(q3))
        if (best_any is None) or (cand["err_mm"] < best_any["err_mm"] - 1e-12): best_any = cand
        if cand["tibia_ok"]:
            if (best_ok is None) or (cand["err_mm"] < best_ok["err_mm"] - 1e-12): best_ok = cand
            elif abs(cand["err_mm"] - best_ok["err_mm"]) <= 1e-12:
                if prefer=="down" and cand["branch"]=="down" and best_ok["branch"]!="down": best_ok = cand
                if prefer=="up"   and cand["branch"]=="up"   and best_ok["branch"]!="up":   best_ok = cand

    best = best_ok if best_ok is not None else best_any
    qdeg = tuple(np.degrees([best["q1"], best["q2"], best["q3"]]))
    best["qdeg"] = qdeg
    best["feasible"] = (best["tibia_ok"] and best["err_mm"] <= tol_mm)
    return best


# ================= World <-> Leg transforms =================
def leg_base_yaw_rad(leg:int) -> float:
    """
    World frame convention: +X forward, +Y left, +Z up.
    Robot layout (front between legs 1 and 4):
      1 = front-right -> ψ = -45°
      2 = back-right  -> ψ = -135°
      3 = back-left   -> ψ = +135°
      4 = front-left  -> ψ = +45°
    """
    if leg == 1: return -np.pi/4    # front-right
    if leg == 2: return -3*np.pi/4  # back-right
    if leg == 3: return  3*np.pi/4  # back-left
    if leg == 4: return  np.pi/4    # front-left
    raise ValueError("leg must be 1..4")


def hip_pitch_world(leg:int, radius:float) -> np.ndarray:
    """Hip-pitch joint position in world frame on circle of given radius."""
    psi = leg_base_yaw_rad(leg)
    return Rz(psi) @ np.array([radius, 0.0, 0.0])

def hip_yaw_world(leg:int, radius:float, o_local:np.ndarray) -> np.ndarray:
    """Hip-yaw world position, given hip-pitch on circle and o (yaw->pitch) in leg frame."""
    psi = leg_base_yaw_rad(leg)
    return hip_pitch_world(leg, radius) - (Rz(psi) @ o_local)

def world_to_leg_yaw_frame(Pw:np.ndarray, leg:int, radius:float, o_local:np.ndarray) -> np.ndarray:
    """
    Convert a world foot-contact target Pw into the leg's hip-yaw frame coordinates
    (the frame expected by leg_ik_with_foot_target).
    """
    psi = leg_base_yaw_rad(leg)
    HYw = hip_yaw_world(leg, radius, o_local)
    return Rz(-psi) @ (Pw - HYw)


class PosGait:
    def __init__(self, controller):
        self.name = "Positioning (Global frame)"
        self.controller = controller
        self.rb = OpenRB()  # serial comms

        # ----- geometry -----
        self.coxa, self.femur, self.tibia, self.foot = 52.0, 107.0, 107.5, 100.0
        # o is still defined in each LEG'S LOCAL frame (yaw->pitch)
        self.o_local = np.array([self.coxa, 0.0, 0.0])
        self.radius_hp = 85.0  # hip-pitch circle radius (mm)

        # joint limits (deg)
        self.limHipYaw, self.limHipPitch, self.limKneePitch = (-90,90), (-120,120), (-120,120)
        self.limAnkle = 20.0   # tibia tilt dev vs vertical (deg)

        # input / selection
        self.inc = 4.0
        self.leg = 1
        self.grip = 0.0

        # ----- per-leg last valid, stored in WORLD coordinates -----
        self.last_valid = {i: {"sol":None, "Pw":None} for i in (1,2,3,4)}

        # ----- seed: take your original leg-1 local target and rotate to world for each leg -----
        seed_local_leg1 = np.array([159.0, 0.0, -207.5])  # this was in leg-1 yaw frame
        for i in (1,2,3,4):
            psi = leg_base_yaw_rad(i)
            HYw = hip_yaw_world(i, self.radius_hp, self.o_local)
            # Bring leg-1 local point into the current leg's local frame (same numbers),
            # then map to world by: Pw = HYw + Rz(psi) * p_local
            Pw = HYw + (Rz(psi) @ seed_local_leg1)
            # Solve IK once to confirm feasibility and cache it
            p_leg = world_to_leg_yaw_frame(Pw, i, self.radius_hp, self.o_local)
            sol = leg_ik_with_foot_target(
                p_leg, self.o_local, self.femur, self.tibia, self.foot,
                self.limAnkle, self.limHipYaw, self.limHipPitch, self.limKneePitch,
                samples=181, tol_mm=5.0, prefer="down"
            )
            if sol["tibia_ok"] and sol["feasible"]:
                self.last_valid[i]["sol"] = sol
                self.last_valid[i]["Pw"]  = Pw

        # current editable global target (start from leg 1's last valid or a default)
        if self.last_valid[1]["Pw"] is not None:
            self.Pw = self.last_valid[1]["Pw"].copy()
        else:
            self.Pw = np.array([ self.radius_hp + 150.0, 0.0, -200.0 ], float)

    def step(self):
        # ----- leg select (X=1, Y=2, B=3, A=4) -----
        if self.controller.is_pressed("X"): self.leg = 1
        elif self.controller.is_pressed("Y"): self.leg = 2
        elif self.controller.is_pressed("B"): self.leg = 3
        elif self.controller.is_pressed("A"): self.leg = 4

        # restore this leg's last Pw if available so each leg remembers its own world target
        if self.last_valid[self.leg]["Pw"] is not None:
            self.Pw = self.last_valid[self.leg]["Pw"].copy()

        # ----- joystick axes -> GLOBAL X,Y,Z editing -----
        trigL = trigR = False
        for i in range(self.controller.joystick.get_numaxes()):
            val = self.controller.joystick.get_axis(i)
            if i in (4,5):  # triggers
                if val >= 0:
                    trigL = trigL or (i==4)
                    trigR = trigR or (i==5)
                continue
            if abs(val) > 0.3:
                if i==0: self.Pw[1] -= val*self.inc   # left stick X -> world Y
                elif i==1: self.Pw[0] -= val*self.inc # left stick Y -> world X
                elif i==3: self.Pw[2] -= val*self.inc # right stick Y -> world Z (up is negative)
        self.grip = -1.0 if (trigL and not trigR) else (+1.0 if (trigR and not trigL) else 0.0)

        # ----- world -> leg yaw frame, solve IK, send command -----
        p_leg = world_to_leg_yaw_frame(self.Pw, self.leg, self.radius_hp, self.o_local)
        sol = leg_ik_with_foot_target(
            p_leg, self.o_local, self.femur, self.tibia, self.foot,
            self.limAnkle, self.limHipYaw, self.limHipPitch, self.limKneePitch,
            samples=181, tol_mm=5.0, prefer="down"
        )

        if sol["tibia_ok"] and sol["feasible"]:
            out = sol
            self.last_valid[self.leg]["sol"] = sol
            self.last_valid[self.leg]["Pw"]  = self.Pw.copy()
        else:
            out = self.last_valid[self.leg]["sol"] if self.last_valid[self.leg]["sol"] else sol
            if self.last_valid[self.leg]["Pw"] is not None:
                self.Pw = self.last_valid[self.leg]["Pw"].copy()

        q1,q2,q3 = out["qdeg"]
        self.rb.send_leg_command(self.leg, q1, q2, q3, self.grip)
        time.sleep(0.05)
