#!/bin/bash

# Create virtual environment if it doesn't exist
if [ ! -d "venv" ]; then
  echo "Creating virtual environment..."
  python3 -m venv --system-site-packages venv
else
  echo "Virtual environment already exists."
fi

# Activate virtual environment
echo "Activating virtual environment..."
source venv/bin/activate

# Upgrade pip
echo "Upgrading pip..."
pip install --upgrade pip

# Install required packages
echo "Installing packages: fastapi uvicorn pillow picamera2 ..."
pip install fastapi uvicorn pillow picamera2

echo ""
echo "Setup complete! To run your app:"
echo "1. Activate the venv: source venv/bin/activate"
echo "2. Start the server: uvicorn main:app --host 0.0.0.0 --port 8000"
echo ""
echo "When done, deactivate the environment with: deactivate"
