from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import numpy as np
import time
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse, StreamingResponse
from PIL import Image
from io import BytesIO
import cv2


app = FastAPI()

# Mount 'static' directory to '/static' URL path
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
    return np.random.uniform(0, 5, (4, 3))

def getTorque():
    return np.random.uniform(0, 100, 4)

def getEEPos():
    return np.random.uniform(-350, 350, (4, 3))

@app.get("/data")
def get_robot_data():
    return {
        "sensors": getSensors().tolist(),
        "torques": getTorque().tolist(),
        "positions": getEEPos().tolist()
    }

# --- FAKE CAMERA STREAM (COLORED SQUARE) ---
def generate_frames(video_path="asteroid.mp4"):
    # Open the video file
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open video {video_path}")

    while True:
        ret, frame = cap.read()
        if not ret:
            # Loop the video
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            continue

        # Convert frame from BGR to RGB
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Encode frame as JPEG
        ret, buffer = cv2.imencode('.jpg', frame_rgb, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        if not ret:
            continue

        # Yield bytes in MJPEG format
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
