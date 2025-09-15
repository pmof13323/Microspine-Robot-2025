class ModeSelector:
    def __init__(self, gaits):
        self.gaits = gaits
        self.current_index = 0

        # Map buttons to gait indices
        # Example: numpad up = 0, left = 1, right = 2
        self.button_map = {
            "init": 0,
            "up": 1,
            "left": 2,
            "right": 3
        }

    @property
    def current_gait(self):
        return self.gaits[self.current_index]

    def select_gait_by_button(self, button_name):
        """Switch mode based on a button press"""
        if button_name in self.button_map:
            idx = self.button_map[button_name]
            if idx != self.current_index:
                self.current_index = idx
                print(f"👉 Switched to gait: {self.current_gait.name}")
