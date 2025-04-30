## UNIVERSITY OF ADELAIDE - 2024
## Engineering Honours Project
## Microspine Anchoring Mechanisms for Robotic Exploration of Small Celestial Bodies
## Amber Pegoli a1799418, Callie Hopwood a1801146, 
## Fida Matin a1798239, Georgia Dallimore a1794409, Grace Gunner a1750264
## 2024s1-EME.Me-EHa-UG-13948

# Main file following State-Machine Diagram 
# Implementing GUI

from gui import Gui     # Import GUI

# Initialise Dashboard
app = Gui()

# Generate all necessary artefacts for dashboard
app.generate_env()

# Setup all motors
app.setup()

# Refresh state machine every 100 milliseconds (10 Hz)
app.after(100, app.state_machine)

# Launch Dashboard
app.mainloop()

# Turn off all motors and write to CSV
app.close()




            
