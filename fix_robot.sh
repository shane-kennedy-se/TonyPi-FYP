#!/bin/bash

# 1. Stop only the specific robot scripts (Safety First)
sudo pkill -f main.py
sudo pkill -f debug_voice.py

# 2. Stop the factory camera app safely
sudo systemctl stop mjpg_streamer

# 3. Clear the Audio Cache (Fixes "Sample format not supported")
echo "Cleaning old audio files..."
rm -rf /home/pi/FYP_Robot/sounds_cache/*.wav

# 4. Fix Permissions (Fixes "Permission denied")
echo "Fixing permissions..."
sudo chmod -R 777 /home/pi/FYP_Robot
sudo chmod 666 /dev/ttyAMA0
sudo chmod 666 /dev/ttyUSB0
sudo chmod +x /home/pi/FYP_Robot/piper_tts/piper

echo "--- RESET COMPLETE ---"