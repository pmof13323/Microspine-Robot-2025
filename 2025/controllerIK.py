# pip install pygame pyserial numpy
import pygame, sys, time, serial
import numpy as np

pygame.init()
pygame.joystick.init()

# ---------- helpers ----------
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

# ---------- geometry & limits ----------
coxa, femur, tibia, foot = 52.0, 107.0, 107.5, 100.0
o = np.array([coxa, 0.0, 0.0])
limHipYaw, limHipPitch, limKneePitch = (-90,90), (-120,120), (-120,120)
limAnkle = 20.0  # ± from vertical

# ---------- initial pose ----------
x, y, z = 159.0, 0.0, -207.5
inc = 0.5
leg = 1
grip = 0.0

# Per-leg memory
last_valid = {i: {"sol":None, "xyz":None} for i in (1,2,3,4)}

# Seed leg 1 memory
seed = leg_ik_with_foot_target([x,y,z], o, femur, tibia, foot,
                               limAnkle, limHipYaw, limHipPitch, limKneePitch,
                               samples=181, tol_mm=5.0, prefer="down")
if seed["tibia_ok"] and seed["feasible"]:
    last_valid[1]["sol"] = seed; last_valid[1]["xyz"] = (x,y,z)

# ---------- serial (Mac) ----------
PORT = "/dev/tty.usbmodem2101"   # <- your device
BAUD = 115200                    # <- match Arduino sketch
ser = serial.Serial(PORT, BAUD, timeout=0.1)
time.sleep(2.0)                  # allow board to enumerate

# ---------- joystick ----------
if pygame.joystick.get_count() == 0:
    print("ERROR: no joystick"); sys.exit(1)
js = pygame.joystick.Joystick(0); js.init()
button_map = {0:"A",1:"B",2:"X",3:"Y"}

try:
    while True:
        pygame.event.pump()

        # Buttons: X=1, Y=2, B=3, A=4
        for i in range(js.get_numbuttons()):
            if js.get_button(i):
                name = button_map.get(i,"")
                prev = leg
                if name=="X": leg=1
                elif name=="Y": leg=2
                elif name=="B": leg=3
                elif name=="A": leg=4
                if leg != prev and last_valid[leg]["xyz"] is not None:
                    x,y,z = last_valid[leg]["xyz"]

        # Axes: sticks to xyz, triggers to grip
        trigL = trigR = False
        for i in range(js.get_numaxes()):
            val = js.get_axis(i)
            if i in (2,5):  # triggers (adjust if your mapping differs)
                if val >= -0.9:
                    trigL = trigL or (i==2)
                    trigR = trigR or (i==5)
                continue
            if abs(val) > 0.3:
                if i==0: y += val*inc   # LS X -> y
                elif i==1: x += val*inc # LS Y -> x (invert if needed)
                elif i==4: z += val*inc # RS Y -> z

        grip = -1.0 if (trigL and not trigR) else (+1.0 if (trigR and not trigL) else 0.0)

        # IK
        p_contact = np.array([x,y,z], float)
        sol = leg_ik_with_foot_target(p_contact, o, femur, tibia, foot,
                                      limAnkle, limHipYaw, limHipPitch, limKneePitch,
                                      samples=181, tol_mm=5.0, prefer="down")

        if sol["tibia_ok"] and sol["feasible"]:
            out = sol; last_valid[leg]["sol"]=sol; last_valid[leg]["xyz"]=(x,y,z)
        else:
            out = last_valid[leg]["sol"] if last_valid[leg]["sol"] is not None else sol
            if last_valid[leg]["xyz"] is not None:
                x,y,z = last_valid[leg]["xyz"]

        q1,q2,q3 = out["qdeg"]
        ser.write((f"L {leg} {q1:.2f} {q2:.2f} {q3:.2f} {grip:.2f}\n").encode())
        pygame.time.wait(50)

except KeyboardInterrupt:
    pass
finally:
    try:
        ser.close()
    except Exception:
        pass
    pygame.quit()
