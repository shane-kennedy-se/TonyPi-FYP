import serial
import os
import time
import subprocess
import hashlib

# --- AUTO-FIX AUDIO PERMISSIONS ---
os.environ["XDG_RUNTIME_DIR"] = "/run/user/1000"

# --- PATHS ---
BASE_DIR = "/home/pi/FYP_Robot"
PIPER_BINARY = os.path.join(BASE_DIR, "piper_tts", "piper")
PIPER_MODEL = os.path.join(BASE_DIR, "piper_tts", "en_US-ryan-medium.onnx")
CACHE_DIR = os.path.join(BASE_DIR, "sounds_cache")

if not os.path.exists(CACHE_DIR):
    os.makedirs(CACHE_DIR, exist_ok=True)

# --- EXACT COMMAND MAPPING (FROM YOUR IMAGES) ---
HEX_COMMANDS = {
    # Interaction
    b'\xaa\x55\x01\x00\xfb': 'Greeting',
    b'\xaa\x55\x02\x00\xfb': 'Sleep',
    b'\xaa\x55\x03\x00\xfb': 'Wake Up',
    
    # Movement (Note: 00 01, not 01 00!)
    b'\xaa\x55\x00\x01\xfb': 'Forward',
    b'\xaa\x55\x00\x02\xfb': 'Back',
    b'\xaa\x55\x00\x03\xfb': 'Turn Left',
    b'\xaa\x55\x00\x04\xfb': 'Turn Right',
    
    # Tasks
    b'\xaa\x55\x00\x05\xfb': 'Peeling',
    b'\xaa\x55\x00\x06\xfb': 'Flip',
    b'\xaa\x55\x00\x07\xfb': 'Insert Label',  # Was wrongly 'pick_up' before
    b'\xaa\x55\x00\x08\xfb': 'Pick Up',       # Was wrongly 'place_box' before
    b'\xaa\x55\x00\x09\xfb': 'Transport',
    b'\xaa\x55\x00\x0a\xfb': 'Stop'
}

def speak(text):
    if not text: return
    print(f"[AI]: {text}")
    
    # 1. Check Cache
    text_hash = hashlib.md5(text.encode('utf-8')).hexdigest()
    wav_path = os.path.join(CACHE_DIR, f"{text_hash}.wav")
    
    if os.path.exists(wav_path):
        os.system(f"aplay -q {wav_path}")
        return

    # 2. Try Piper
    if os.path.exists(PIPER_BINARY):
        try:
            clean = text.replace('"', '')
            cmd = f'echo "{clean}" | {PIPER_BINARY} --model {PIPER_MODEL} --output_file {wav_path}'
            subprocess.run(cmd, shell=True, check=True)
            os.system(f"aplay -q {wav_path}")
            return
        except: pass

    # 3. Fallback
    os.system(f'espeak "{text}"')

class WonderEcho:
    def __init__(self):
        self.ser = None
        # Try finding the port aggressively
        ports = ['/dev/ttyAMA0', '/dev/ttyUSB0', '/dev/serial0']
        for port in ports:
            if os.path.exists(port):
                try:
                    # Your image code used 115200, but standard is usually 9600.
                    # We try 9600 first as it's safer for these modules.
                    self.ser = serial.Serial(port, 9600, timeout=0.05)
                    print(f"[VOICE] Connected to {port}")
                    break
                except: pass
        
        if not self.ser:
            print("[VOICE ERROR] Hardware not found!")

    def get_command(self):
        if not self.ser: return None
        try:
            if self.ser.in_waiting > 0:
                # Read byte by byte to find header AA
                byte = self.ser.read(1)
                if byte == b'\xaa':
                    rest = self.ser.read(4)
                    full = byte + rest
                    if full in HEX_COMMANDS:
                        return HEX_COMMANDS[full]
        except: pass
        return None