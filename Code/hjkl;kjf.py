import RPi.GPIO as gpio
import time

TRIG = 5
ECHO = 6

gpio.setmode(gpio.BCM)
gpio.setup(TRIG, gpio.OUT)
gpio.setup(ECHO, gpio.IN)

def measure_distance():
    gpio.output(TRIG, False)
    time.sleep(0.05)
    gpio.output(TRIG, True)
    time.sleep(0.0001)
    gpio.output(TRIG, False)

    pulse_start = time.time()
    timeout = pulse_start + 0.04
    while gpio.input(ECHO) == 0:
        pulse_start = time.time()
        if pulse_start > timeout:
            print("Timeout waiting for ECHO to go high")
            return 999

    pulse_end = time.time()
    timeout = pulse_end + 0.04
    while gpio.input(ECHO) == 1:
        pulse_end = time.time()
        if pulse_end > timeout:
            print("Timeout waiting for ECHO to go low")
            return 999

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance, 2)
    if distance <= 2 or distance >= 400:
        print("Out of range or no object detected")
        return 999
    return distance

try:
    while True:
        dist = measure_distance()
        print(f"Distance: {dist} cm")
        time.sleep(1)
except KeyboardInterrupt:
    gpio.cleanup()