import numpy as np
import time
from TransmitData import OpenRB

# --- helpers (pulled from your standalone script) ---
def clamp(x, lo, hi): return np.minimum(np.maximum(x, lo), hi)

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
    Rz = np.array([[c,-s,0],[s,c,0],[0,0,1]])
    dp = Rz @ d; r, z = float(np.hypot(dp[0], dp[1])), float(dp[2])

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


class PosGait:
    def __init__(self, controller):
        self.name = "Positioning"
        self.controller = controller
        self.rb = OpenRB()  # serial comms

        # geometry
        self.coxa, self.femur, self.tibia, self.foot = 52.0, 107.0, 107.5, 100.0
        self.o = np.array([self.coxa, 0.0, 0.0])
        self.limHipYaw, self.limHipPitch, self.limKneePitch = (-90,90), (-120,120), (-120,120)
        self.limAnkle = 20.0

        # initial state
        self.x, self.y, self.z = 159.0, 0.0, -207.5
        self.inc = 4
        self.leg = 1
        self.grip = 0.0
        self.last_valid = {i: {"sol":None, "xyz":None} for i in (1,2,3,4)}

        # seed leg 1
        seed = leg_ik_with_foot_target([self.x,self.y,self.z], self.o,
                                       self.femur, self.tibia, self.foot,
                                       self.limAnkle, self.limHipYaw,
                                       self.limHipPitch, self.limKneePitch,
                                       samples=181, tol_mm=5.0, prefer="down")
        if seed["tibia_ok"] and seed["feasible"]:
            self.last_valid[1]["sol"] = seed
            self.last_valid[1]["xyz"] = (self.x,self.y,self.z)

    def step(self):
        # Check button presses (leg select)
        if self.controller.is_pressed("X"): self.leg = 1
        elif self.controller.is_pressed("Y"): self.leg = 2
        elif self.controller.is_pressed("B"): self.leg = 3
        elif self.controller.is_pressed("A"): self.leg = 4

        if self.last_valid[self.leg]["xyz"] is not None:
            self.x, self.y, self.z = self.last_valid[self.leg]["xyz"]

        # Axes → xyz and triggers
        trigL = trigR = False
        for i in range(self.controller.joystick.get_numaxes()):
            val = self.controller.joystick.get_axis(i)
            if i in (4,5):  # triggers
                if val >= 0:
                    trigL = trigL or (i==4)
                    trigR = trigR or (i==5)
                continue
            if abs(val) > 0.3:
                if i==0: self.y += val*self.inc
                elif i==1: self.x += val*self.inc
                elif i==3: self.z -= val*self.inc

        self.grip = -1.0 if (trigL and not trigR) else (+1.0 if (trigR and not trigL) else 0.0)

        # Solve IK
        p_contact = np.array([self.x,self.y,self.z], float)
        sol = leg_ik_with_foot_target(p_contact, self.o, self.femur, self.tibia, self.foot,
                                      self.limAnkle, self.limHipYaw, self.limHipPitch, self.limKneePitch,
                                      samples=181, tol_mm=5.0, prefer="down")

        if sol["tibia_ok"] and sol["feasible"]:
            out = sol
            self.last_valid[self.leg]["sol"] = sol
            self.last_valid[self.leg]["xyz"] = (self.x,self.y,self.z)
        else:
            out = self.last_valid[self.leg]["sol"] if self.last_valid[self.leg]["sol"] else sol
            if self.last_valid[self.leg]["xyz"] is not None:
                self.x,self.y,self.z = self.last_valid[self.leg]["xyz"]

        q1,q2,q3 = out["qdeg"]
        self.rb.send_leg_command(self.leg, q1, q2, q3, self.grip)
        time.sleep(0.05)
