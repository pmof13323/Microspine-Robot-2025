from Position import *

class AngleGait:
    def __init__(self):
        self.name = "Angle control"
    def step(self):
        print("Running AngleGait...")

class WalkGait:
    def __init__(self):
        self.name = "Walking cycle"
    def step(self):
        print("Running WalkGait...")

class inital:
    def __init__(self,OpenRB):
        self.name="inital pose"

        self.rb = OpenRB
                # Define your initial positions (servo IDs 1–12)
        initial_positions = [
            (1, 2047), (2, 2047), (3, 3063),
            (4, 2047), (5, 2047), (6, 3063),
            (7, 2047), (8, 2047), (9, 3063),
            (10, 2047), (11, 2047), (12, 3063),
        ]

        # Send them as a SYNC packet
        self.rb.send_sync_positions(initial_positions)
        print("on the d-pad press up for position gait, left for angle gait, and right for walk gait")

    def step(self):
        pass
