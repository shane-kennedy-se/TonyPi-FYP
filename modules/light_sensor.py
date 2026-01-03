import RPi.GPIO as GPIO

class LightSensor:
    def __init__(self, pin=24):
        self.pin = pin
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.IN)

    def is_dark(self):
        # Returns True if it is dark (Sensor Output High = Dark usually)
        # You might need to flip this to 'return GPIO.input(self.pin) == 0' depending on your specific sensor dial
        return GPIO.input(self.pin) == 1