import numpy as np
import time
from TransmitData import OpenRB
from Position import (
    leg_ik_with_foot_target,
    world_to_leg_yaw_frame,
    hip_yaw_world,
    leg_base_yaw_rad,
    Rz,
)

# Utility
def deg_to_dxl(angle_deg, min_deg=-180.0, max_deg=180.0, resolution=4095):
    return int(np.clip((angle_deg - min_deg) / (max_deg - min_deg) * resolution, 0, resolution))

# Class to provide access to End-effector positional controls
class EEPlanner:
    def __init__(self, ee_positions: dict, geom: dict):
        self.ee = ee_positions  # shared dict {leg: [x,y,z]} (updated elsewhere)
        self.geom = geom
        self.o_local = np.array([self.geom["coxa"], 0.0, 0.0], float)

    def current_world_pos(self, leg: int) -> np.ndarray:
        p = self.ee.get(leg)
        if p is None:
            # fallback if missing
            return np.array([self.geom["radius_hp"] + 150.0, 0.0, -200.0], float)
        return np.array(p, float)

    def solve_world_target(self, leg: int, Pw_target: np.ndarray):
        """Run the same IK solver used elsewhere for a given world-frame target."""
        p_leg = world_to_leg_yaw_frame(Pw_target, leg, self.geom["radius_hp"], self.o_local)
        sol = leg_ik_with_foot_target(
            p_leg,
            self.o_local,
            self.geom["femur"],
            self.geom["tibia"],
            self.geom["foot"],
            tibia_dev_deg=20.0,
            lim1_deg=(-90.0, 90.0),
            lim2_deg=(-110.0, 110.0),
            lim3_deg=(-110.0, 130.0),
            samples=30,
            tol_mm=5.0,
            prefer="down",
        )
        feasible = bool(sol["tibia_ok"] and sol["feasible"])
        return sol, feasible



class WalkGait:
    def __init__(self, controller, rb: OpenRB, Angles: dict, eePos: dict, eeLoad: dict):
        self.name = "Walking cycle"
        self.controller = controller
        self.rb = rb              # serial comms
        self.angles = Angles      # {leg: [q1,q2,q3] in deg}
        self.ee = eePos           # shared dict {leg: [x,y,z]
        self.currents = eeLoad

        # Geometry (match your PosGait numbers)
        self.geom = dict(coxa=52.0, femur=107.5, tibia=93.401, foot=112.124, radius_hp=98.427)
        self.o_local = np.array([self.geom["coxa"], 0.0, 0.0], float)

        # Limits/params
        self.dead = 0.30
        self.incrementer = 75.0     # mm to lift
        self.sticky = False          # EE “stick” flag
        self.maxCurrent = 88.0 #mA

        # Planner (composition)
        self.planner = EEPlanner(self.ee, self.geom)

        # One-time init flag
        self._initialized = False

        # Body transform state
        self.body_T = np.zeros(3)
        self.body_anchors = {i: None for i in (1, 2, 3, 4)}

        # Direction placeholder
        self.direction = None

    # small helper used below
    def _send_leg_angles(self, leg: int, qdeg_tuple):
        q1, q2, q3 = qdeg_tuple
        q3 = -q3  # knee sign flip per your convention
        base_id = (leg - 1) * 3
        sync_targets = [
            (base_id + 1, deg_to_dxl(q1)),
            (base_id + 2, deg_to_dxl(q2)),
            (base_id + 3, deg_to_dxl(q3)),
        ]
        self.angles[leg] = [q1, q2, q3]
        self.rb.send_sync_positions(sync_targets)
        time.sleep(0.001)

    def _capture_body_anchors(self):
        # Freeze current feet world positions to act as fixed anchors for body translation.
        for i in (1, 2, 3, 4):
            Pw = self.planner.current_world_pos(i)
            self.body_anchors[i] = Pw.copy()
        self.body_T[:] = 0.0

    # Automatically appends EE
        # Takes in a leg number, and a 'sticky' flag
        # Returns a Bool
            # True if sensors give good reading after EE current reached
            # False if sensors give bad reading after EE current reached
    def appendEE(self, legNum, stickyFlag):
        # Checking for sticky flag
        if not stickyFlag:
            return True
        
        # Start adhering EE
        current = 0.0
        while current < self.maxCurrent:
            current = float(self.currents.get(legNum, 0.0))
            # send sync write to activate leg ee
            sync_targets = [(legNum + 12, 1.0)]
            self.rb.send_sync_positions(sync_targets)
            # run a timer to determine how long it moves for
            time.sleep(0.02)
            # safety: break if no update path
            break

        # Once current is reached, run if statement
            # if all sensors are green
        sensorVoltage = 13.2 #Assuming sensor voltage of 4.4V per sensor
        if sensorVoltage > 13.2: #Assuming sensor voltage of 4.4V per sensor
            sync_targets = [(legNum + 12, 0.0)]
            self.rb.send_sync_positions(sync_targets)
            return True
                
            # if less than 3 sensors are green 
        if sensorVoltage <= 13.2:
            # send sync write to deactivate leg ee, runnning for a duration as determined by the above timer
            sync_targets = [(legNum + 12, -1.0)]
            self.rb.send_sync_positions(sync_targets)
            time.sleep(10.0)

            # one ee full disengaged
            sync_targets = [(legNum + 12, 0.0)]
            self.rb.send_sync_positions(sync_targets)
            return False

        return True

    def releaseEE(self, legNum, stickyFlag):
        # Checking for sticky flag
        if not stickyFlag:
            return True
        
        sync_targets = [(legNum + 12, -1.0)]
        self.rb.send_sync_positions(sync_targets)
        time.sleep(10.0)

        sync_targets = [(legNum + 12, 0.0)]
        self.rb.send_sync_positions(sync_targets)
        return True

    # Completes the follow steps to move a limb
      # 1. Translate up in Z from current EE location
      # 2. Move EE across to new location
      # 3. Translate EE down in z to new location
        # Takes in leg number, new position
        # Returns a Bool
            # True if successful
            # False if failure occurs
    def moveLimb(self, legNum, goalPos, lift_mm=None):

        lift = float(lift_mm if lift_mm is not None else self.incrementer)
        goalPos = np.array(goalPos, float)

        # Read xyz of requested EE
        legXYZ = self.planner.current_world_pos(legNum)

        # Translate the EE +z 
        legXYZLift = legXYZ.copy()
        legXYZLift[2] = legXYZ[2] + lift
        solLift, feasibleLift = self.planner.solve_world_target(legNum, legXYZLift)
        if not feasibleLift:
            return False
        self._send_leg_angles(legNum, solLift["qdeg"])
        self.ee[legNum] = [float(legXYZLift[0]), float(legXYZLift[1]), float(legXYZLift[2])]
        time.sleep(2.0)
            
        # Move EE to requested x and y location
        legXYZTrans = np.array([goalPos[0], goalPos[1], legXYZLift[2]], float)
        solTrans, feasibleTrans = self.planner.solve_world_target(legNum, legXYZTrans)
        if not feasibleTrans:
            # try smaller lift as a quick fallback
            cur = legXYZ  # current before lift
            retry = legXYZTrans.copy(); retry[2] = cur[2] + 0.5*lift
            solTransB, okTransB = self.planner.solve_world_target(legNum, retry)
            if not okTransB:
                return False
            solTrans, legXYZTrans = solTransB, retry
        self._send_leg_angles(legNum, solTrans["qdeg"])
        self.ee[legNum] = [float(legXYZTrans[0]), float(legXYZTrans[1]), float(legXYZTrans[2])]
        time.sleep(5.0)

        # Translate the EE -z
        legXYZGoal = goalPos.copy()
        solGoal, feasibleGoal = self.planner.solve_world_target(legNum, legXYZGoal)
        if not feasibleGoal:
            return False
        self._send_leg_angles(legNum, solGoal["qdeg"])
        self.ee[legNum] = [float(legXYZGoal[0]), float(legXYZGoal[1]), float(legXYZGoal[2])]
        time.sleep(2.0)

        # Body move happens outside this function.
        return True

    # ---- Functional BODY TRANSLATION (feet anchored) ----
    def translateBody(self, dT):
        # Capture anchors first time
        if self.body_anchors[1] is None:
            self._capture_body_anchors()

        dT = np.array(dT, float)
        T_candidate = self.body_T + dT
        all_ok = True
        sols = {}
        pw_eff_map = {}

        # Check all legs first
        for i in (1, 2, 3, 4):
            anchor = self.body_anchors[i] if self.body_anchors[i] is not None else self.planner.current_world_pos(i)
            Pw_eff = anchor - T_candidate
            sol, ok = self.planner.solve_world_target(i, Pw_eff)
            sols[i] = (sol, ok, Pw_eff)
            pw_eff_map[i] = Pw_eff
            if not ok:
                all_ok = False
                break

        if not all_ok:
            return False

        # Commit body move: send all legs in one SYNC and update states
        self.body_T = T_candidate
        sync_targets = []
        for i in (1, 2, 3, 4):
            sol, _, Pw_eff = sols[i]
            q1, q2, q3 = sol["qdeg"]
            q3 = -q3
            base_id = (i - 1) * 3
            sync_targets.extend([
                (base_id + 1, deg_to_dxl(q1)),
                (base_id + 2, deg_to_dxl(q2)),
                (base_id + 3, deg_to_dxl(q3)),
            ])
            # keep global EE positions in sync (world frame)
            self.ee[i] = [float(Pw_eff[0]), float(Pw_eff[1]), float(Pw_eff[2])]
            self.angles[i] = [q1, q2, q3]

        self.rb.send_sync_positions(sync_targets)
        time.sleep(0.001)
        return True

    # runs a single iteration in the direction of the stick (N, E, S, W only)
        # Takes in a direction
        # Returns a Bool
            # True when cycle is done
            # False if failure occurs
    def moveThisDirection(self, direction):

        # Given the requested direction, calculate the EE positions
        if direction == 'N':
            leg1XYZ = [175.0, -145.26, -207.5]
            leg2XYZ = [0.0, -145.26, -207.5]
            leg3XYZ = [0.0, 145.26, -207.5]
            leg4XYZ = [175.0, 145.26, -207.5]
            body_dT = np.array([+20.0, 0.0, 0.0])
        elif direction == 'S':
            leg1XYZ = [0.0, -145.26, -207.5]
            leg2XYZ = [-175.0, -145.26, -207.5]
            leg3XYZ = [-175.0, 145.26, -207.5]
            leg4XYZ = [0.0, 145.26, -207.5]
            body_dT = np.array([-20.0, 0.0, 0.0])
        elif direction == 'W':
            leg1XYZ = [160.0, 0.0, -207.5]
            leg2XYZ = [-160.0, 0.0, -207.5]
            leg3XYZ = [-160.0, 175.0, -207.5]
            leg4XYZ = [160.0, 175.0, -207.5]
            body_dT = np.array([0.0, +20.0, 0.0])
        elif direction == 'E':
            leg1XYZ = [160.0, 175.0, -207.5]
            leg2XYZ = [-160.0, 175.0, -207.5]
            leg3XYZ = [-160.0, 0.0, -207.5]
            leg4XYZ = [160.0, 0.0, -207.5]
            body_dT = np.array([0.0, -20.0, 0.0])
        else:
            return False
    
        # Start gait cycle
        for leg, xyz in zip((1, 4, 2, 3), (leg1XYZ, leg4XYZ, leg2XYZ, leg3XYZ)):
            if not self.moveLimb(leg, xyz, lift_mm=20.0):
                return False
            # EE adhesion (placeholder)
            attempts, ok = 0, self.appendEE(leg, self.sticky)
            while not ok and attempts < 3:
                time.sleep(0.05)
                ok = self.appendEE(leg, self.sticky)
                attempts += 1
            if not ok:
                return False

        # Translate the Body (now functional)
        moved = self.translateBody(body_dT)
        if not moved:
            # optional: try a smaller translation if the big one isn't feasible
            moved = self.translateBody(0.5 * body_dT)

        return moved

    # moves the body to its initil position
        # returns true when complete
        # returns false if failure occurs 
    def moveToInitial(self):
        leg1Home = [159.0, -145.0, -205.0]
        leg2Home = [-159.0, -145.0, -205.0]
        leg3Home = [-159.0, 145.0, -205.0]
        leg4Home = [159.0, 145.0, -205.0]
            
        # Move the robot back into its initial position    
        for leg, xyz in zip((1, 4, 2, 3), (leg1Home, leg4Home, leg2Home, leg3Home)):
            if not self.moveLimb(leg, xyz):
                return False
            # EE adhesion (placeholder)
            attempts, ok = 0, self.appendEE(leg, self.sticky)
            while not ok and attempts < 3:
                time.sleep(0.05)
                ok = self.appendEE(leg, self.sticky)
                attempts += 1
            if not ok:
                return False

        # capture anchors after homing
        self._capture_body_anchors()
        return True
            

    def step(self):
        print("Running WalkGait...")
        sync_targets = []

        # Move robot into initial psoition and adhering
        if not self._initialized:
            ok = self.moveToInitial()
            if not ok:
                print("[WalkGait] Failed to move to initial posture.")
                return
            self._initialized = True

        # Reading in joystick
        js = getattr(self.controller, "joystick", None)

        if js is not None:
            for i in range(js.get_numaxes()):
                val = js.get_axis(i)

                if abs(val) > self.dead:
                    if i == 0: # North/South
                        if val > 0: # North
                            self.direction = 'N'
                            print('N')
                        if val < 0: # South
                            self.direction = 'S'
                            print('S')

                    elif i == 1: #East/West
                        if val > 0: # East
                            self.direction = 'E'
                            print('E')
                        if val < 0: # West
                            self.direction = 'W'
                            print('W')

        if self.direction is not None:
            ok = self.moveThisDirection(self.direction)
            if not ok:
                print(f"[WalkGait] Move in {self.direction} failed; attempting to continue.")
        else:
            # Idle: optionally keep posture / micro-corrections, etc.
            pass

        print(f"\n")
        print(f" Walking Control Mode ")
        print("+-------------------------------------------------------+")
        print(" ")
        print(" ")
        print("                    Under Construction                   ")
        print(" ")
        print(" ")
        print("+-------------------------------------------------------+")
        self.rb.send_sync_positions(sync_targets)
        print("+-------------------------------------------------------+")
