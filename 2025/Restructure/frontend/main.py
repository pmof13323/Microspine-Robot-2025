
from fastapi import FastAPI, Response
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import random
from io import BytesIO
from PIL import Image, ImageDraw
from fastapi.responses import StreamingResponse
from picamera2 import Picamera2
from PIL import Image
from io import BytesIO
import numpy as np
import time
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from sensor_read import SensorArray
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
import numpy as np
from sensor_loop import SensorBackgroundReader  # <-- import your loop class

app = FastAPI()
app.mount("/static", StaticFiles(directory="static"), name="static")
OPENRB_STATE_FILE = "openrb_state.json"

# Start background sensor reading
sensor_reader = SensorBackgroundReader()
sensor_reader.start()

@app.get("/")
async def root():
    return FileResponse("static/frontend.html")

# CORS setup
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# --- MOCK DATA FUNCTIONS ---
def getSensors():
    return np.random.uniform(0, 5, (4, 3))

def getEEPos():
    return np.random.uniform(-350, 350, (4, 3))

@app.get("/data")
def get_robot_data():
    """
    Serve the latest OpenRB motor states (separated lists for load, velocity, position)
    along with sensor data and timestamp.
    """
    # --- Sensor data ---
    sensor_data = sensor_reader.get_data()
    sensors = sensor_data["sensors"] if sensor_data else []
    timestamp = sensor_data["timestamp"] if sensor_data else None

    # --- OpenRB motor state ---
    loads, vels, positions = [], [], []
    if os.path.exists(OPENRB_STATE_FILE):
        try:
            with open(OPENRB_STATE_FILE, "r") as f:
                openrb_state = json.load(f)
            # Separate values into lists
            for motor_id in sorted(openrb_state.keys(), key=int):
                motor = openrb_state[motor_id]
                loads.append(motor.get("load", 0))
                vels.append(motor.get("vel", 0))
                positions.append(motor.get("pos", 0))
        except json.JSONDecodeError:
            loads, vels, positions = [], [], []

    return {
        "sensors": sensors,
        "loads": loads,
        "velocities": vels,
        "positions": positions,
        "timestamp": timestamp
    }


# Configure camera once
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(main={"size": (640, 360)}))
picam2.start()

def generate_frames():
    while True:
        frame = picam2.capture_array()
        img = Image.fromarray(frame)

        # Convert to RGB if it's RGBA
        if img.mode == "RGBA":
            img = img.convert("RGB")

        buf = BytesIO()
        img.save(buf, format="JPEG", quality=70)  # Compress to keep it fast
        buf.seek(0)
        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" +
            buf.read() + b"\r\n"
        )
        time.sleep(0.033)  # ~30 FPS (adjust if needed)


@app.get("/camera/stream")
def camera_stream():
    return StreamingResponse(generate_frames(), media_type="multipart/x-mixed-replace; boundary=frame")


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)