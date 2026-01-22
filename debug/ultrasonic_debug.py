import lgpio
import time

h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(h, 28)  # TRIG
lgpio.gpio_claim_input(h, 29)   # ECHO
lgpio.gpio_write(h, 28, 0)
time.sleep(0.5)

print("Testing ultrasonic sensor timing...")
print("-" * 40)

for i in range(10):
    # Send trigger pulse
    lgpio.gpio_write(h, 28, 1)
    time.sleep(0.00001)  # 10us
    lgpio.gpio_write(h, 28, 0)
    
    # Wait for echo to go HIGH
    wait_start = time.time()
    while lgpio.gpio_read(h, 29) == 0:
        if time.time() - wait_start > 0.05:
            print(f"Test {i+1}: TIMEOUT waiting for echo HIGH")
            break
    else:
        # Echo went HIGH - measure duration
        start = time.time()
        while lgpio.gpio_read(h, 29) == 1:
            if time.time() - start > 0.05:
                print(f"Test {i+1}: TIMEOUT waiting for echo LOW (stuck HIGH)")
                break
        else:
            stop = time.time()
            duration_us = (stop - start) * 1_000_000
            distance = (stop - start) * 34300 / 2
            print(f"Test {i+1}: Echo duration = {duration_us:.0f}us, Distance = {distance:.1f}cm")
    
    time.sleep(0.2)

lgpio.gpiochip_close(h)
print("-" * 40)
print("Done. If all readings are similar regardless of obstacles,")
print("the sensor hardware is likely faulty.")