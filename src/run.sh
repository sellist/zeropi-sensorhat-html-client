#!/bin/bash

sudo apt update
sudo apt upgrade -y

sudo apt install -y python3-pip python3-venv python3-smbus python3-pil i2c-tools

if [ ! -d "venv" ]; then
    echo "Creating virtual environment..."
    python3 -m venv venv
fi

echo "Activating virtual environment.. (This may take a while)"
source venv/bin/activate

echo "Installing requirements..."
pip install -r requirements.txt

echo "Running main.py..."
python3 main.py