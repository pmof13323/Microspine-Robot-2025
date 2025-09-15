
from fastapi import FastAPI, Response
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import numpy as np
import random
from io import BytesIO
from PIL import Image, ImageDraw

app = FastAPI()

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
    return np.random.uniform(-5, 5, 4)

def getEEPos():
    return np.random.uniform(-350, 350, (4, 3))

@app.get("/data")
def get_robot_data():
    return {
        "sensors": getSensors().tolist(),
        "torques": getTorque().tolist(),
        "positions": getEEPos().tolist()
    }

@app.get("/camera")
def get_camera_frame():
    # Placeholder: generates a simple image with random color background
    img = Image.new("RGB", (640, 360), (random.randint(0,255), random.randint(0,255), random.randint(0,255)))
    draw = ImageDraw.Draw(img)
    draw.text((10, 10), "Camera Placeholder", fill=(255, 255, 255))
    buf = BytesIO()
    img.save(buf, format="JPEG")
    return Response(content=buf.getvalue(), media_type="image/jpeg")

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)