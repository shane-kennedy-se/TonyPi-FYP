import RPi.GPIO as GPIO

class LightSensor:
    def __init__(self, pin=23):
        self.pin = pin
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.IN)

    def is_dark(self):
        """
        Returns True if the sensor sees Darkness.
        (Change '== 1' to '== 0' if your sensor is inverted)
        """
        return GPIO.input(self.pin) == 1

    def cleanup(self):
        GPIO.cleanup()