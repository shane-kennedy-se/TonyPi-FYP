import serial
import time
import os
import subprocess
import hashlib

# --- 1. PIPER CONFIGURATION ---
# Path to your Piper executable and model
PIPER_BINARY = "/home/pi/piper/piper" 
PIPER_MODEL = "/home/pi/piper/en_US-lessac-medium.onnx"
CACHE_DIR = "/home/pi/TonyPi/sounds_cache"

if not os.path.exists(CACHE_DIR):
    os.makedirs(CACHE_DIR, exist_ok=True)

# --- 2. HEX COMMAND MAPPING (From your Spreadsheet) ---
# We map the raw bytes to the text commands main.py expects
HEX_COMMANDS = {
    b'\xaa\x55\x01\x00\xfb': 'greeting',             # Welcome
    b'\xaa\x55\x02\x00\xfb': 'sleep',                # Break
    b'\xaa\x55\x03\x00\xfb': 'wake_up',              # Yes Boss
    b'\xaa\x55\x00\x05\xfb': 'peeling',              # Peeling
    b'\xaa\x55\x00\x06\xfb': 'flip_over',            # Flipping
    b'\xaa\x55\x00\x07\xfb': 'label_insertion',      # Inserting
    b'\xaa\x55\x00\x08\xfb': 'pick_up_cardboard',    # Picking up
    b'\xaa\x55\x00\x09\xfb': 'transport_cardboard',  # Transport
    b'\xaa\x55\x00\x0a\xfb': 'stop',                 # Stop
    # Movement commands
    b'\xaa\x55\x00\x01\xfb': 'forward',
    b'\xaa\x55\x00\x02\xfb': 'back',
    b'\xaa\x55\x00\x03\xfb': 'turn_left',
    b'\xaa\x55\x00\x04\xfb': 'turn_right',
}

class WonderEcho:
    def __init__(self, baud=9600):
        self.ser = None
        
        # Hunt for the hardware on valid ports
        ports_to_try = ['/dev/ttyUSB0', '/dev/ttyAMA0', '/dev/serial0']
        for port in ports_to_try:
            if os.path.exists(port):
                try:
                    self.ser = serial.Serial(port, baud, timeout=0.1)
                    print(f"[VOICE] Success! Connected to {port}")
                    break
                except Exception as e:
                    print(f"[VOICE] Found {port} but failed: {e}")
        
        if self.ser is None:
            print("[VOICE] WARNING: Hardware not found.")

    def get_command(self):
        """
        Reads 5 bytes of Hex data and matches it to a command.
        """
        if not self.ser: return None
        
        try:
            # The spreadsheet shows commands are exactly 5 bytes long
            if self.ser.in_waiting >= 5:
                
                # 1. Sync: Read byte-by-byte until we find the Header (0xAA)
                byte = self.ser.read(1)
                if byte != b'\xaa':
                    return None # Skip noise

                # 2. Read the remaining 4 bytes of the frame
                rest = self.ser.read(4)
                full_frame = byte + rest
                
                # 3. Check against our dictionary
                if full_frame in HEX_COMMANDS:
                    return HEX_COMMANDS[full_frame]
                else:
                    print(f"[VOICE] Unknown Hex: {full_frame}")
                
        except Exception as e:
            print(f"[VOICE ERROR] {e}")
            
        return None

    def close(self):
        if self.ser: self.ser.close()

def speak(text):
    """
    Piper TTS with Caching.
    """
    if not text: return
    
    # Generate unique filename
    text_hash = hashlib.md5(text.encode('utf-8')).hexdigest()
    wav_path = os.path.join(CACHE_DIR, f"{text_hash}.wav")

    # A. Play Cached File (Fast)
    if os.path.exists(wav_path):
        os.system(f"aplay -q {wav_path}")
        return

    # B. Generate New File (Piper)
    print(f"[SPEAK] Generating: '{text}'")
    if os.path.exists(PIPER_BINARY) and os.path.exists(PIPER_MODEL):
        try:
            clean_text = text.replace('"', '').replace("'", "")
            cmd = f'echo "{clean_text}" | {PIPER_BINARY} --model {PIPER_MODEL} --output_file {wav_path}'
            subprocess.run(cmd, shell=True)
            os.system(f"aplay -q {wav_path}")
            return
        except Exception as e:
            print(f"[TTS ERROR] Piper failed: {e}")

    # C. Fallback
    try:
        os.system(f'echo "{text}" | festival --tts')
    except:
        pass