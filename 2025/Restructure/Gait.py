from Position import *

import time

class AngleGait:
    def __init__(self, controller,OpenRB):
        self.name = "Angle control"
        self.controller = controller
        self.rb = OpenRB  # serial comms

        # Variables
        self.leg  = 1
        self.inc  = 3.0
        self.grip = 0.0
        self.dead = 0.30

        # Angle Storage
        self.angle_storage = {
            1: [0.0, 0.0, 90.0],
            2: [0.0, 0.0, 90.0],
            3: [0.0, 0.0, 90.0],
            4: [0.0, 0.0, 90.0],
        }
        # CANgle Referemce
        self.current_angles = self.angle_storage[self.leg]

    # Leg switcher helper
    def switch_leg(self):
        """Switch active leg based on button presses; preserve stored angles."""
        new_leg = self.leg
        c = getattr(self, "controller", None)
        if c is not None:
            if c.is_pressed("X"): new_leg = 4
            elif c.is_pressed("Y"): new_leg = 1
            elif c.is_pressed("B"): new_leg = 2
            elif c.is_pressed("A"): new_leg = 3

        if new_leg != self.leg:
            self.leg = new_leg
            # Re-point to the stored list for that leg (no resets)
            self.current_angles = self.angle_storage.setdefault(self.leg, [0.0, 0.0, 0.0])

    def step(self):
        """Update stored angles from joystick; send targets for the active leg only."""
        self.switch_leg()

        sync_targets = []
        js = getattr(self.controller, "joystick", None)
        trigL = trigR = False

        if js is not None:
            for i in range(js.get_numaxes()):
                val = js.get_axis(i)

                if i in (4, 5):
                    if val >= 0:
                        trigL = trigL or (i == 4)
                        trigR = trigR or (i == 5)
                    continue

                if abs(val) > self.dead:
                    # Map axes -> joints: 0->q1 (yaw), 1->q2 (hip pitch), 3->q3 (knee pitch)
                    if i == 0:
                        self.current_angles[0] -= val * self.inc
                    elif i == 1:
                        self.current_angles[1] -= val * self.inc
                    elif i == 3:
                        self.current_angles[2] += val * self.inc

        self.grip = -1.0 if (trigL and not trigR) else (+1.0 if (trigR and not trigL) else 0.0)

        q1, q2, q3 = self.current_angles
        base_id = (self.leg - 1) * 3
        try:
            sync_targets.extend([
                (base_id + 1, deg_to_dxl(q1)),
                (base_id + 2, deg_to_dxl(q2)),
                (base_id + 3, deg_to_dxl(q3)),
            ])
        except NameError:
            raise RuntimeError("deg_to_dxl(q_deg) is not defined in this module. Please provide it.")
        print(f"\n")
        print(f" Angular Control Mode")
        print("+-------------------------------------+-----------------+")
        print(f" Controlling Leg Number {self.leg}             | Grip Mode: {self.grip:.2f}")
        print("+----------+--------------------------+-----------------+")
        print(f" Angles    | HY: {q1:.2f}deg,  HP: {q2:.2f}deg, KP: {q3:.2f}deg")
        print("+----------+--------------------------------------------+")
        print(" ")
        print("+-------------------------------------------------------+")
        if sync_targets and hasattr(self, "rb"):
            self.rb.send_sync_positions(sync_targets)
        print("+-------------------------------------------------------+")
        time.sleep(0.03)  # keep loop quick but not spammy



class WalkGait:
    def __init__(self, controller,OpenRB):
        self.name = "Walking cycle"
        self.controller = controller
        self.rb = OpenRB  # serial comms
    def step(self):
        print("Running WalkGait...")
        sync_targets = []



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

class inital:
    def __init__(self,OpenRB):
        self.name="inital pose"

        self.rb = OpenRB
                # Define your initial positions (servo IDs 1–12)
        self.initial_positions = [
            (1, 2047), (2, 2047), (3, 3063),
            (4, 2047), (5, 2047), (6, 3063),
            (7, 2047), (8, 2047), (9, 3063),
            (10, 2047), (11, 2047), (12, 3063),
        ]

        # Send them as a SYNC packet
        self.rb.send_sync_positions(self.initial_positions)
        print("on the d-pad press up for position gait, left for angle gait, and right for walk gait")

    def step(self):
        print(f"\n")
        print(f" ")
        print("+-------------------------------------------------------+")
        print("               Robot is in initial stance                ")
        print(" ")
        print("          Please use the D-Pad to select a mode:         ")
        print("        Up: Positional          Down: Pre-programmed     ")
        print("        Left: Angular           Right: Walk              ")
        print("+-------------------------------------------------------+")
        self.rb.send_sync_positions(self.initial_positions)
        print("+-------------------------------------------------------+")