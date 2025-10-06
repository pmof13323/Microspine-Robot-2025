
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
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import numpy as np
import time
import json
import socket
import threading
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse, StreamingResponse
import cv2
from collections import deque

# Start background sensor reading
sensor_reader = SensorBackgroundReader()
sensor_reader.start()

app = FastAPI()

# ---- GLOBAL STATE ----
latest_openrb_state = {}    # store last received motor data (JSON messages)
latest_timestamp = 0.0
active_quadrant_index = 0   # integer expected by front-end (0..4 per your UI comment)
mode_code = 0               # integer expected by front-end (0..4)
MAXMOTORTORQUE = 88

LEG_ORDER = [3, 0, 2, 1] 

def reorder_list(data_list):
    """Reorder a list according to LEG_ORDER (only if it's the right length)."""
    if not isinstance(data_list, list) or len(data_list) != len(LEG_ORDER):
        return data_list
    return [data_list[i] for i in LEG_ORDER]

def build_return_payload(sensors, torques_grouped, positions_grouped, coord,
                         latest_timestamp, active_quadrant_index, mode_code, serial_buffer):
    return {
        "sensors": reorder_list(sensors),
        "torques": reorder_list(torques_grouped),
        "positions": reorder_list(positions_grouped),
        "timestamp": latest_timestamp,
        "activeQuadrantIndex": active_quadrant_index,
        "modeCode": mode_code,
        "serial": list(serial_buffer),
        "coord": reorder_list(coord)  # reorder legs before sending
    }

# small circular buffer of non-JSON / textual serial lines to show in the UI
serial_buffer = deque(maxlen=400)

# ---- SOCKET READER (runs in background) ----
# This will connect to the OpenRB server and handle:
#  - JSON lines -> parsed into latest_openrb_state
#  - "modeN" -> update mode_code
#  - "legN"  -> update active_quadrant_index
#  - other text -> appended to serial_buffer
def socket_listener(host="127.0.0.1", port=5002):
    global latest_openrb_state, latest_timestamp, mode_code, active_quadrant_index, serial_buffer
    while True:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            print(f"[FastAPI] Connecting to OpenRBReader socket at {host}:{port} ...")
            s.connect((host, port))
            print("[FastAPI] Connected to OpenRBReader server.")
            buffer = ""
            while True:
                try:
                    data = s.recv(4096).decode(errors="ignore")
                    if not data:
                        print("[FastAPI] Socket closed by server.")
                        break
                    buffer += data
                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        line = line.strip()
                        if not line:
                            continue

                        low = line.lower()

                        # mode messages like "mode1" or "mode 1" (robust)
                        if low.startswith("mode"):
                            # extract digits from the line (robust to "mode:2" or "mode 2")
                            digits = "".join(ch for ch in low if ch.isdigit())
                            if digits:
                                try:
                                    code = int(digits)
                                    mode_code = code
                                    serial_buffer.append(f"[MODE] {line}")
                                    print(f"[FastAPI] Received mode -> {mode_code}")
                                except ValueError:
                                    serial_buffer.append(f"[BAD MODE] {line}")
                            else:
                                serial_buffer.append(f"[BAD MODE] {line}")
                            continue

                        # leg messages like "leg3" or "leg 3"
                        if low.startswith("leg"):
                            digits = "".join(ch for ch in low if ch.isdigit())
                            if digits:
                                try:
                                    idx = int(digits)
                                    active_quadrant_index = idx
                                    serial_buffer.append(f"LEG {line}")
                                    print(f"[FastAPI] Received leg -> {active_quadrant_index}")
                                except ValueError:
                                    serial_buffer.append(f"[BAD LEG] {line}")
                            else:
                                serial_buffer.append(f"[BAD LEG] {line}")
                            continue

                        # attempt JSON parse (motor states)
                        try:
                            parsed = json.loads(line)
                            serial_buffer.append(line)
                            # only accept dict-like JSON for motor state
                            if isinstance(parsed, dict):
                                latest_openrb_state = parsed
                                latest_timestamp = time.time()
                            else:
                                serial_buffer.append(line)
                        except json.JSONDecodeError:
                            # not JSON: keep for serial UI
                            serial_buffer.append(line)

                except (ConnectionResetError, OSError) as e:
                    print("[FastAPI] Connection lost during recv:", e)
                    break

        except ConnectionRefusedError:
            print("[FastAPI] Waiting for OpenRBReader server...")
            time.sleep(1)
            continue
        except OSError as e:
            print("[FastAPI] Socket error:", e)
            time.sleep(1)
            continue
        finally:
            try:
                s.close()
            except:
                pass

        # wait a moment and then try reconnecting
        time.sleep(1)


# Start socket listener in background
threading.Thread(target=socket_listener, daemon=True).start()

# ---- FASTAPI SETUP ----
app.mount("/static", StaticFiles(directory="static"), name="static")

@app.get("/")
async def root():
    return FileResponse("static/frontend.html")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # allow all origins for testing
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# --- MOCK DATA FUNCTIONS ---
def getSensors():
    return np.random.uniform(0, 5, (4, 3)).tolist()

def torque_percent_conversion(value, max, min ):
    return (value/(max-min)*100) if (max-min) != 0 else 0

def pos_tic_angle_convert(angle, inv):
    return inv*(-360/4096*angle+180)

def convert_coord_object_to_array(coord_dict):
    if not isinstance(coord_dict, dict):
        return []

    # Sort keys numerically just in case they're strings like "1", "2"
    keys = sorted(coord_dict.keys(), key=lambda k: int(k))

    coord_array = []
    for key in keys:
        c = coord_dict[key]
        coord_array.append([float(c["x"]), float(c["y"]), float(c["z"])])
    return coord_array

# --- OPENRB MOTOR DATA ---
@app.get("/data")
def get_robot_data():
    # --- Sensor data ---
    sensor_data = sensor_reader.get_data()
    sensors = sensor_data["sensors"] if sensor_data else []
    timestamp = sensor_data["timestamp"] if sensor_data else None

    loads, vels, positions = [], [], []

    motors = {}
    if latest_openrb_state:
        motors = latest_openrb_state.get("motors", {})

    if motors:
        motor_keys = sorted(motors.keys(), key=lambda x: int(x))
        for k in motor_keys:
            motor = motors[k]
            loads.append(
                torque_percent_conversion(np.abs(motor.get("load", 0)), MAXMOTORTORQUE, 0)
            )
            vels.append(motor.get("vel", 0))
            positions.append(pos_tic_angle_convert(motor.get("pos", 0), 1))

    # Group into 4 quadrants x 3 motors
    torques_grouped, positions_grouped = [], []
    quadrant_count, motors_per_quadrant = 4, 3

    for q in range(quadrant_count):
        start, end = q * motors_per_quadrant, (q + 1) * motors_per_quadrant
        limb_torques = loads[start:end] if end <= len(loads) else [0] * motors_per_quadrant

        # Add the extra torque (motor number = q + 12)
        extra_idx = q + 12
        extra_torque = loads[extra_idx] if extra_idx < len(loads) else 0
        limb_torques.append(extra_torque)
        torques_grouped.append(limb_torques)

        limb_positions = positions[start:end] if end <= len(positions) else [0] * motors_per_quadrant
        positions_grouped.append(limb_positions)

    coord = convert_coord_object_to_array(latest_openrb_state.get("legs", {})) 

    return build_return_payload(
        sensors,
        torques_grouped,
        positions_grouped,
        coord,
        latest_timestamp,
        active_quadrant_index,
        mode_code,
        serial_buffer
    )


# --- CAMERA STREAM ---
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
