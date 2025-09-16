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



def deg_to_dxl(angle_deg, min_deg=-180.0, max_deg=180.0, resolution=4095):
    return int(np.clip((angle_deg - min_deg) / (max_deg - min_deg) * resolution, 0, resolution))

class PosGait:
    def __init__(self, controller,OpenRB):
        self.name = "Positioning (Global frame)"
        self.controller = controller
        self.rb = OpenRB  # serial comms

        # ----- geometry -----
        self.coxa, self.femur, self.tibia, self.foot = 52.0, 107.0, 107.5, 100.0
        self.o_local = np.array([self.coxa, 0.0, 0.0])
        self.radius_hp = 85.0

        self.limHipYaw, self.limHipPitch, self.limKneePitch = (-90,90), (-120,120), (-120,120)
        self.limAnkle = 20.0

        self.inc = 3.0
        self.dead = 0.30
        self.leg = 1
        self.grip = 0.0
        self.mode = 'leg'
        self.body_T = np.zeros(3)
        self.body_anchors = {i: None for i in (1,2,3,4)}
        self.last_valid = {i: {"sol":None, "Pw":None} for i in (1,2,3,4)}

        seed_local_leg1 = np.array([159.0, 0.0, -207.5])
        for i in (1,2,3,4):
            psi = leg_base_yaw_rad(i)
            HYw = hip_yaw_world(i, self.radius_hp, self.o_local)
            Pw = HYw + (Rz(psi) @ seed_local_leg1)
            p_leg = world_to_leg_yaw_frame(Pw, i, self.radius_hp, self.o_local)
            sol = leg_ik_with_foot_target(
                p_leg, self.o_local, self.femur, self.tibia, self.foot,
                self.limAnkle, self.limHipYaw, self.limHipPitch, self.limKneePitch,
                samples=30, tol_mm=5.0, prefer="down"
            )
            if sol["tibia_ok"] and sol["feasible"]:
                self.last_valid[i]["sol"] = sol
                self.last_valid[i]["Pw"]  = Pw

        self.Pw = self.last_valid[1]["Pw"].copy() if self.last_valid[1]["Pw"] is not None else np.array([self.radius_hp+150.0,0.0,-200.0],float)

    def _solve_leg_for_world_target(self, leg:int, Pw_world:np.ndarray):
        """Return (sol, feasible_bool) for a given leg and world foot contact Pw_world."""
        p_leg = world_to_leg_yaw_frame(Pw_world, leg, self.radius_hp, self.o_local)
        sol = leg_ik_with_foot_target(
            p_leg, self.o_local, self.femur, self.tibia, self.foot,
            self.limAnkle, self.limHipYaw, self.limHipPitch, self.limKneePitch,
            samples=30, tol_mm=5.0, prefer="down"
        )
        feasible = bool(sol["tibia_ok"] and sol["feasible"])
        return sol, feasible

    def _capture_body_anchors(self):
        """Freeze current feet world positions to act as fixed anchors for body mode."""
        for i in (1,2,3,4):
            if self.last_valid[i]["Pw"] is not None:
                self.body_anchors[i] = self.last_valid[i]["Pw"].copy()
            else:
                psi = leg_base_yaw_rad(i)
                HYw = hip_yaw_world(i, self.radius_hp, self.o_local)
                self.body_anchors[i] = HYw + (Rz(psi) @ np.array([159.0, 0.0, -207.5]))
        # zero body translation when entering body mode
        self.body_T[:] = 0.0

    def _read_axes_xy_z(self):
        """Read sticks as (dx, dy, dz) increments in WORLD frame based on self.inc & deadzone."""
        dx = dy = dz = 0.0
        js = self.controller.joystick
        # Left stick X (axis 0) -> +Y; Left stick Y (axis 1) -> +X
        ax0 = js.get_axis(0) if js.get_numaxes() > 0 else 0.0
        ax1 = js.get_axis(1) if js.get_numaxes() > 1 else 0.0
        ax3 = js.get_axis(3) if js.get_numaxes() > 3 else 0.0  # right stick Y -> Z
        if abs(ax0) > self.dead: dy -= ax0 * self.inc  # matches your existing sign convention
        if abs(ax1) > self.dead: dx -= ax1 * self.inc
        if abs(ax3) > self.dead: dz -= ax3 * self.inc
        return np.array([dx, dy, dz], float)

    def step(self):
        sync_targets = []  # accumulate (id, pos) pairs here

        if self.controller.is_pressed("Right_bumper"):
            if self.mode != 'body':
                self.mode = 'body'
                self._capture_body_anchors()
                print("👉 Body movement mode (feet anchored)")
        elif self.controller.is_pressed("Left_bumper"):
            if self.mode != 'leg':
                self.mode = 'leg'
                print("👉 Leg movement mode (edit one foot)")

        if self.mode == 'leg':
            # select leg
            if self.controller.is_pressed("X"): self.leg = 4
            elif self.controller.is_pressed("Y"): self.leg = 1
            elif self.controller.is_pressed("B"): self.leg = 2
            elif self.controller.is_pressed("A"): self.leg = 3

            if self.last_valid[self.leg]["Pw"] is not None:
                self.Pw = self.last_valid[self.leg]["Pw"].copy()

            js = self.controller.joystick
            trigL = trigR = False
            for i in range(js.get_numaxes()):
                val = js.get_axis(i)
                if i in (4,5):
                    if val >= 0:
                        trigL = trigL or (i==4)
                        trigR = trigR or (i==5)
                    continue
                if abs(val) > self.dead:
                    if i==0: self.Pw[1] -= val*self.inc
                    elif i==1: self.Pw[0] -= val*self.inc
                    elif i==3: self.Pw[2] -= val*self.inc
            self.grip = -1.0 if (trigL and not trigR) else (+1.0 if (trigR and not trigL) else 0.0)

            sol, feasible = self._solve_leg_for_world_target(self.leg, self.Pw)
            if feasible:
                self.last_valid[self.leg]["sol"] = sol
                self.last_valid[self.leg]["Pw"]  = self.Pw.copy()
            else:
                sol = self.last_valid[self.leg]["sol"] if self.last_valid[self.leg]["sol"] else sol
                if self.last_valid[self.leg]["Pw"] is not None:
                    self.Pw = self.last_valid[self.leg]["Pw"].copy()

            q1, q2, q3 = sol["qdeg"]
            q3 = -q3
            # Example mapping: each leg has 3 servos, assign IDs in order
            base_id = (self.leg - 1) * 3
            sync_targets.extend([
                (base_id+1, deg_to_dxl(q1)),
                (base_id+2, deg_to_dxl(q2)),
                (base_id+3, deg_to_dxl(q3)),
            ])

        else:  # BODY MODE
            dT = self._read_axes_xy_z()
            if np.linalg.norm(dT) > 0.0:
                T_candidate = self.body_T + dT
                all_ok = True
                sols = {}
                Pws={}
                for i in (1,2,3,4):
                    Pw_eff = self.body_anchors[i] - T_candidate
                    sol, ok= self._solve_leg_for_world_target(i, Pw_eff)
                    sols[i] = (sol, ok, Pw_eff)
                    Pws[i] = ()
                    if not ok:
                        all_ok = False
                        break
                if all_ok:
                    self.body_T = T_candidate
                    for i in (1,2,3,4):
                        sol, _, Pws = sols[i]
                        self.last_valid[i]["sol"] = sol
                        self.last_valid[i]["Pw"]  = Pws
                        q1,q2,q3 = sol["qdeg"]
                        q3 = -q3
                        base_id = (i - 1) * 3
                        sync_targets.extend([
                            (base_id+1, deg_to_dxl(q1)),
                            (base_id+2, deg_to_dxl(q2)),
                            (base_id+3, deg_to_dxl(q3)),
                        ])
            else:
                for i in (1,2,3,4):
                    sol = self.last_valid[i]["sol"]
                    if sol is None:
                        continue
                    q1,q2,q3 = sol["qdeg"]
                    q3 = -q3
                    base_id = (i - 1) * 3
                    sync_targets.extend([
                        (base_id+1, deg_to_dxl(q1)),
                        (base_id+2, deg_to_dxl(q2)),
                        (base_id+3, deg_to_dxl(q3)),
                    ])

        # 🔑 Send everything in one go
        if sync_targets:
            print(f"\n")
            print(f" Positional Control Mode")
            print(f"+-------------------------------------+-----------------+")
            print(f" Controlling Leg Number {self.leg}             | Grip Mode: {self.grip:.2f}")
            print(f"+----------+--------------------------+-----------------+")
            print(f" Angles    | HY: {q1:.2f}deg,  HP: {q2:.2f}deg, KP: {q3:.2f}deg")
            print(f"+----------+--------------------------------------------+")
            print(f" Positions | x = {self.Pw[0]:.1f}mm, y = {self.Pw[1]:.1f}mm, z = {self.Pw[2]:.1f}mm")
            print(f"+----------+--------------------------------------------+")
            self.rb.send_sync_positions(sync_targets)
            print(f"+-------------------------------------------------------+")
            time.sleep(0.03)  # reduce serial spam but keep fast loop
