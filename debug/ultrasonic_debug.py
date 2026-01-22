import lgpio
import time

h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(h, 28)  # TRIG
lgpio.gpio_claim_input(h, 29)   # ECHO

# Check if ECHO pin is stuck
print(f"ECHO pin state: {lgpio.gpio_read(h, 29)}")  # Should be 0 normally

lgpio.gpiochip_close(h)