import serial
import time
import os
import subprocess
import hashlib

# --- CONFIGURATION (UPDATED) ---
# Assuming 'FYP_Robot' is in the home folder: /home/pi/FYP_Robot
# If it is somewhere else, change '/home/pi' to that location.
PROJECT_ROOT = "/home/pi/FYP_Robot" 

# Pointing to your specific folder structure
PIPER_BINARY = os.path.join(PROJECT_ROOT, "piper_tts", "piper")
# Using the 'Ryan' model you requested
PIPER_MODEL = os.path.join(PROJECT_ROOT, "piper_tts", "en_US-ryan-medium.onnx")

CACHE_DIR = os.path.join(PROJECT_ROOT, "sounds_cache")

if not os.path.exists(CACHE_DIR):
    os.makedirs(CACHE_DIR, exist_ok=True)

# --- EXACT COMMAND MAPPING ---
HEX_COMMANDS = {
    b'\xaa\x55\x01\x00\xfb': 'greeting',
    b'\xaa\x55\x02\x00\xfb': 'sleep',
    b'\xaa\x55\x03\x00\xfb': 'hello_tony',
    b'\xaa\x55\x00\x01\xfb': 'forward',
    b'\xaa\x55\x00\x02\xfb': 'back',
    b'\xaa\x55\x00\x03\xfb': 'turn_left',
    b'\xaa\x55\x00\x04\xfb': 'turn_right',
    b'\xaa\x55\x00\x05\xfb': 'do_diecut_peeling',
    b'\xaa\x55\x00\x06\xfb': 'do_sheet_flipover',
    b'\xaa\x55\x00\x07\xfb': 'do_label_insertion',
    b'\xaa\x55\x00\x08\xfb': 'pick_up_cardboard',
    b'\xaa\x55\x00\x09\xfb': 'transport_cardboard',
    b'\xaa\x55\x00\x0a\xfb': 'stop'
}

class WonderEcho:
    def __init__(self, baud=9600):
        self.ser = None
        ports = ['/dev/ttyUSB0', '/dev/ttyAMA0', '/dev/serial0']
        for port in ports:
            if os.path.exists(port):
                try:
                    self.ser = serial.Serial(port, baud, timeout=0.05)
                    print(f"[VOICE] Connected to hardware at {port}")
                    break
                except: pass
        
        if self.ser is None:
            print("[VOICE] WARNING: Hardware not found.")

    def get_command(self):
        if not self.ser: return None
        try:
            if self.ser.in_waiting >= 5:
                byte = self.ser.read(1)
                if byte == b'\xaa':
                    rest = self.ser.read(4)
                    full_frame = byte + rest
                    if full_frame in HEX_COMMANDS:
                        return HEX_COMMANDS[full_frame]
            self.ser.flushInput()
        except: pass
        return None

    def close(self):
        if self.ser: self.ser.close()

def speak(text):
    if not text: return
    
    # Check Cache
    text_hash = hashlib.md5(text.encode('utf-8')).hexdigest()
    wav_path = os.path.join(CACHE_DIR, f"{text_hash}.wav")
    if os.path.exists(wav_path):
        os.system(f"aplay -q {wav_path}")
        return

    # Generate with Piper (Ryan)
    if os.path.exists(PIPER_BINARY) and os.path.exists(PIPER_MODEL):
        try:
            clean = text.replace('"', '').replace("'", "")
            # Added --output_file explicitly
            cmd = f'echo "{clean}" | {PIPER_BINARY} --model {PIPER_MODEL} --output_file {wav_path}'
            subprocess.run(cmd, shell=True, check=True)
            os.system(f"aplay -q {wav_path}")
            return
        except Exception as e:
            print(f"[TTS] Piper Error: {e}")
    else:
        print(f"[TTS] ERROR: Piper not found at {PIPER_BINARY}")
    
    # Fallback
    try: os.system(f'echo "{text}" | festival --tts')
    except: pass