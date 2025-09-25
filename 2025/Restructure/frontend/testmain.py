# server.py  (replace your current FastAPI file with this)
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

app = FastAPI()

# ---- GLOBAL STATE ----
latest_openrb_state = {}    # store last received motor data (JSON messages)
latest_timestamp = 0.0
active_quadrant_index = 0   # integer expected by front-end (0..4 per your UI comment)
mode_code = 0               # integer expected by front-end (0..4)
MAXMOTORTORQUE = 88

# small circular buffer of non-JSON / textual serial lines to show in the UI
serial_buffer = deque(maxlen=400)

# ---- SOCKET READER (runs in background) ----
# This will connect to the OpenRB server and handle:
#  - JSON lines -> parsed into latest_openrb_state
#  - "modeN" -> update mode_code
#  - "legN"  -> update active_quadrant_index
#  - other text -> appended to serial_buffer
def socket_listener(host="127.0.0.1", port=5000):
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
                                    serial_buffer.append(f"[LEG] {line}")
                                    print(f"[FastAPI] Received leg -> {active_quadrant_index}")
                                except ValueError:
                                    serial_buffer.append(f"[BAD LEG] {line}")
                            else:
                                serial_buffer.append(f"[BAD LEG] {line}")
                            continue

                        # attempt JSON parse (motor states)
                        try:
                            parsed = json.loads(line)
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

# --- OPENRB MOTOR DATA ---
@app.get("/data")
def get_robot_data():
    loads, vels, positions = [], [], []

    if latest_openrb_state:
        # latest_openrb_state keys are expected numeric-ish (strings or ints)
        motor_keys = sorted(latest_openrb_state.keys(), key=lambda x: int(x))
        for k in motor_keys:
            motor = latest_openrb_state[k]
            loads.append(torque_percent_conversion(np.abs((motor.get("load", 0))), MAXMOTORTORQUE, 0))
            vels.append(motor.get("vel", 0))
            positions.append(pos_tic_angle_convert((motor.get("pos", 0)), 1))

    # Group into 4 quadrants x 3 motors (and add extra motor per limb as 4th item)
    torques_grouped = []
    positions_grouped = []
    quadrant_count = 4
    motors_per_quadrant = 3
    for q in range(quadrant_count):
        start = q * motors_per_quadrant
        end = start + motors_per_quadrant
        limb_torques = loads[start:end] if end <= len(loads) else [0] * motors_per_quadrant

        # Add the extra torque (motor number = q + 12)
        extra_idx = q + 12
        extra_torque = loads[extra_idx] if extra_idx < len(loads) else 0
        limb_torques.append(extra_torque)

        torques_grouped.append(limb_torques)

        # Positions remain 3 per limb
        limb_positions = positions[start:end] if end <= len(positions) else [0] * motors_per_quadrant
        positions_grouped.append(limb_positions)

    sensors = getSensors()

    return {
        "sensors": sensors,
        "torques": torques_grouped,
        "positions": positions_grouped,
        "timestamp": latest_timestamp,
        # keys expected by your frontend:
        "activeQuadrantIndex": active_quadrant_index,
        "modeCode": mode_code,
        # serial console lines (array)
        "serial": list(serial_buffer)
    }

# --- CAMERA STREAM ---
def generate_frames(video_path="asteroid.mp4"):
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open video {video_path}")

    while True:
        ret, frame = cap.read()
        if not ret:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            continue

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        ret, buffer = cv2.imencode('.jpg', frame_rgb, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        if not ret:
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" +
            buffer.tobytes() + b"\r\n"
        )
        time.sleep(0.03)  # ~30 FPS

@app.get("/camera/stream")
def camera_stream():
    return StreamingResponse(generate_frames(), media_type="multipart/x-mixed-replace; boundary=frame")

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
