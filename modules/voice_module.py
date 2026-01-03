import serial
import os
import time
import subprocess
import hashlib

# --- CONFIG ---
os.environ["XDG_RUNTIME_DIR"] = "/run/user/1000"

BASE_DIR = "/home/pi/FYP_Robot"
PIPER_BINARY = os.path.join(BASE_DIR, "piper_tts", "piper")
PIPER_MODEL = os.path.join(BASE_DIR, "piper_tts", "en_US-ryan-medium.onnx")
CACHE_DIR = os.path.join(BASE_DIR, "sounds_cache")

if not os.path.exists(CACHE_DIR):
    os.makedirs(CACHE_DIR, exist_ok=True)

# --- EXACT MAPPING FROM YOUR SPREADSHEET ---
HEX_COMMANDS = {
    b'\xaa\x55\x01\x00\xfb': 'Greeting',
    b'\xaa\x55\x02\x00\xfb': 'Sleep',
    b'\xaa\x55\x03\x00\xfb': 'Wake Up',       # HELLO-TONY -> "Yes boss?"
    
    b'\xaa\x55\x00\x01\xfb': 'Forward',
    b'\xaa\x55\x00\x02\xfb': 'Back',
    b'\xaa\x55\x00\x03\xfb': 'Turn Left',
    b'\xaa\x55\x00\x04\xfb': 'Turn Right',
    
    b'\xaa\x55\x00\x05\xfb': 'Peeling',
    b'\xaa\x55\x00\x06\xfb': 'Flip',
    b'\xaa\x55\x00\x07\xfb': 'Insert Label',
    b'\xaa\x55\x00\x08\xfb': 'Pick Up',
    b'\xaa\x55\x00\x09\xfb': 'Transport',
    b'\xaa\x55\x00\x0a\xfb': 'Stop'
}

def speak(text):
    if not text: return
    print(f"[ROBOT SAYS]: {text}")
    
    # 1. Try Piper First
    try:
        text_hash = hashlib.md5(text.encode('utf-8')).hexdigest()
        wav_path = os.path.join(CACHE_DIR, f"{text_hash}.wav")
        
        if not os.path.exists(wav_path):
            if os.path.exists(PIPER_BINARY):
                clean = text.replace('"', '').replace("'", "")
                cmd = f'echo "{clean}" | {PIPER_BINARY} --model {PIPER_MODEL} --output_file {wav_path}'
                subprocess.run(cmd, shell=True, check=True)
                
        if os.path.exists(wav_path):
            # Try playing with standard player, fallback to plug driver if format fails
            os.system(f"aplay -q {wav_path} || aplay -D plughw:1,0 -q {wav_path}")
            return
            
    except Exception:
        pass # Silently fail over to fallback

    # 2. Fallback (Espeak) - Runs if Piper fails
    os.system(f'espeak -s150 "{text}" 2>/dev/null')

class WonderEcho:
    def __init__(self):
        self.ser = None
        ports = ['/dev/ttyAMA0', '/dev/ttyUSB0', '/dev/serial0']
        for port in ports:
            if os.path.exists(port):
                try:
                    self.ser = serial.Serial(port, 9600, timeout=0.1)
                    print(f"[VOICE] Connected to {port}")
                    break
                except: pass

    def get_command(self):
        if not self.ser: return None
        try:
            if self.ser.in_waiting > 0:
                byte = self.ser.read(1)
                if byte == b'\xaa':
                    rest = self.ser.read(4)
                    full = byte + rest
                    if full in HEX_COMMANDS:
                        return HEX_COMMANDS[full]
        except: pass
        return None