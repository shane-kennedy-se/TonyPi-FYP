#!/bin/bash
echo "--- 1. KILLING BACKGROUND GHOSTS ---"
sudo pkill -f python3
sudo killall mjpg_streamer
sudo systemctl stop mjpg_streamer

echo "--- 2. UNLOCKING HARDWARE PORTS ---"
# This fixes the "Permission Denied" on the Voice Module
sudo chmod 666 /dev/ttyAMA0
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/video0

echo "--- 3. FIXING FILE PERMISSIONS ---"
# This makes sure every script you own is executable
cd /home/pi/FYP_Robot
chmod +x *.py
chmod +x modules/*.py
chmod +x piper_tts/piper

echo "--- 4. INSTALLING DEPENDENCIES FOR ROOT ---"
# Ensures sudo python3 can find your libraries
sudo pip3 install ultralytics pyserial --break-system-packages

echo "--- DONE! YOU CAN NOW RUN MAIN.PY ---"