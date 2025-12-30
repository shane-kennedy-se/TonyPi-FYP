import serial
import os
import time
import subprocess
import hashlib

# --- 1. CRITICAL: AUTO-FIX PERMISSIONS ---
# This stops the "XDG_RUNTIME_DIR" error forever.
os.environ["XDG_RUNTIME_DIR"] = "/run/user/1000"

# --- CONFIGURATION ---
# We use absolute paths to prevent "File Not Found" errors
BASE_DIR = "/home/pi/FYP_Robot"
PIPER_BINARY = os.path.join(BASE_DIR, "piper_tts", "piper")
PIPER_MODEL = os.path.join(BASE_DIR, "piper_tts", "en_US-ryan-medium.onnx")
CACHE_DIR = os.path.join(BASE_DIR, "sounds_cache")

if not os.path.exists(CACHE_DIR):
    os.makedirs(CACHE_DIR, exist_ok=True)

# --- COMMANDS (From your Image) ---
cmd_dict = {
    b"\xaa\x55\x01\x00\xfb": 'greeting',
    b"\xaa\x55\x02\x00\xfb": 'sleep',
    b"\xaa\x55\x03\x00\xfb": 'wake_up',      
    b"\xaa\x55\x00\x01\xfb": 'forward',
    b"\xaa\x55\x00\x02\xfb": 'back',
    b"\xaa\x55\x00\x03\xfb": 'turn_left',
    b"\xaa\x55\x00\x04\xfb": 'turn_right',
    b"\xaa\x55\x00\x05\xfb": 'peeling',      
    b"\xaa\x55\x00\x06\xfb": 'flip_over',    
    b"\xaa\x55\x00\x07\xfb": 'label_insertion',      
    b"\xaa\x55\x00\x08\xfb": 'pick_up_cardboard',    
    b"\xaa\x55\x00\x09\xfb": 'transport_cardboard',      
    b"\xaa\x55\x00\x0A\xfb": 'stop'   
}

def speak(text):
    """Speaks using Piper (Ryan), with automatic fallback."""
    if not text: return
    print(f"[SPEAK] {text}")

    # 1. Check Cache
    text_hash = hashlib.md5(text.encode('utf-8')).hexdigest()
    wav_path = os.path.join(CACHE_DIR, f"{text_hash}.wav")
    
    if os.path.exists(wav_path):
        os.system(f"aplay -q {wav_path}")
        return

    # 2. Try Piper
    try:
        if os.path.exists(PIPER_BINARY) and os.path.exists(PIPER_MODEL):
            clean_text = text.replace('"', '').replace("'", "")
            cmd = f'echo "{clean_text}" | {PIPER_BINARY} --model {PIPER_MODEL} --output_file {wav_path}'
            subprocess.run(cmd, shell=True, check=True)
            os.system(f"aplay -q {wav_path}")
            return
    except Exception as e:
        print(f"[ERROR] Piper failed: {e}")

    # 3. Fallback to Espeak (The "Old Reliable")
    os.system(f'espeak -ven+f3 -s140 -a100 "{text}" &')

class WonderEcho:
    def __init__(self, baud=9600):
        self.ser = None
        # Try all ports
        ports = ['/dev/ttyUSB0', '/dev/ttyAMA0', '/dev/serial0']
        for port in ports:
            if os.path.exists(port):
                try:
                    self.ser = serial.Serial(port, baud, timeout=0.05)
                    self.ser.flushInput()
                    print(f"[VOICE] Connected at {port}")
                    break
                except: pass

    def get_command(self):
        if not self.ser: return None
        try:
            if self.ser.in_waiting >= 5:
                # Read exactly 5 bytes as per your protocol
                data = self.ser.read(5)
                if data in cmd_dict:
                    return cmd_dict[data]
                # If data is misaligned, flush it
                self.ser.flushInput()
        except: pass
        return None