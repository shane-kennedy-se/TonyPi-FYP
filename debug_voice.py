#!/usr/bin/python3
import serial
import time
import os
import subprocess
import hashlib

# --- AUTO-FIX AUDIO PERMISSIONS ---
os.environ["XDG_RUNTIME_DIR"] = "/run/user/1000"

# --- PATHS ---
BASE_DIR = "/home/pi/FYP_Robot"
PIPER_BINARY = os.path.join(BASE_DIR, "piper_tts", "piper")
PIPER_MODEL = os.path.join(BASE_DIR, "piper_tts", "en_US-ryan-medium.onnx")
CACHE_DIR = os.path.join(BASE_DIR, "sounds_cache")

# --- EXACT COMMANDS FROM YOUR IMAGE ---
HEX_COMMANDS = {
    b'\xaa\x55\x01\x00\xfb': 'Greeting',
    b'\xaa\x55\x02\x00\xfb': 'Sleep',
    b'\xaa\x55\x03\x00\xfb': 'Wake Up',
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
    print(f"[SPEAKING]: {text}")
    wav_path = os.path.join(CACHE_DIR, "temp.wav")
    
    # Try Piper first
    if os.path.exists(PIPER_BINARY):
        cmd = f'echo "{text}" | {PIPER_BINARY} --model {PIPER_MODEL} --output_file {wav_path}'
        try:
            subprocess.run(cmd, shell=True, check=True)
            os.system(f"aplay -q {wav_path}")
            return
        except: pass
    
    # Fallback
    os.system(f'espeak "{text}"')

def test():
    print("--- VOICE DEBUGGER ---")
    ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=0.1) # Try ttyAMA0 first for TonyPi
    speak("Ready.")
    
    while True:
        if ser.in_waiting:
            byte = ser.read(1)
            if byte == b'\xaa':
                full = byte + ser.read(4)
                hex_str = " ".join([f"{b:02X}" for b in full])
                print(f"[HEX]: {hex_str}", end=" -> ")
                
                if full in HEX_COMMANDS:
                    cmd = HEX_COMMANDS[full]
                    print(f"MATCH: {cmd}")
                    speak(f"Command {cmd}")
                else:
                    print("UNKNOWN")
        time.sleep(0.01)

if __name__ == "__main__":
    test()