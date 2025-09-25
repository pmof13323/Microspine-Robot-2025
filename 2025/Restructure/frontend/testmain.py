from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import numpy as np
import time
import json
import os
import socket
import threading
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse, StreamingResponse
import cv2

app = FastAPI()

# ---- GLOBAL STATE ----
latest_openrb_state = {}  # store last received motor data
latest_timestamp = 0.0

MAXMOTORTORQUE = 880

# ---- SOCKET READER (runs in background) ----
def socket_listener(host="127.0.0.1", port=5000):
    global latest_openrb_state, latest_timestamp
    print(f"[FastAPI] Connecting to OpenRBReader socket at {host}:{port} ...")
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    connected = False
    while not connected:
        try:
            s.connect((host, port))
            connected = True
        except ConnectionRefusedError:
            print("[FastAPI] Waiting for OpenRBReader server...")
            time.sleep(1)

    print("[FastAPI] Connected to OpenRBReader server.")
    buffer = ""
    while True:
        try:
            data = s.recv(4096).decode()
            if not data:
                print("[FastAPI] Socket closed by server.")
                break

            buffer += data
            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                try:
                    latest_openrb_state = json.loads(line)
                    latest_timestamp = time.time()
                except json.JSONDecodeError:
                    print(f"[FastAPI] Bad JSON: {line}")

        except (ConnectionResetError, OSError):
            print("[FastAPI] Connection lost. Reconnecting...")
            time.sleep(1)
            return socket_listener(host, port)  # reconnect

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
    return (value/(max-min)*100)

def pos_tic_angle_convert(angle, inv):
    return inv*(-360/4096*angle+180)

# --- OPENRB MOTOR DATA ---
@app.get("/data")
def get_robot_data():
    loads, vels, positions = [], [], []

    if latest_openrb_state:
        motor_keys = sorted(latest_openrb_state.keys(), key=int)
        for k in motor_keys:
            motor = latest_openrb_state[k]
            loads.append(torque_percent_conversion(np.abs((motor.get("load", 0))),MAXMOTORTORQUE,0))
            vels.append(motor.get("vel", 0))
            positions.append(pos_tic_angle_convert((motor.get("pos", 0)),1))

    # Group into 4 quadrants x 3 motors
    torques_grouped = []
    positions_grouped = []
    quadrant_count = 4
    motors_per_quadrant = 3
    for q in range(quadrant_count):
        # Normal 3 torques
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
        "serial": []
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
