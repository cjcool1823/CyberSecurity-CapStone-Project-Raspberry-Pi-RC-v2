import RPi.GPIO as gpio
import time

MOTOR_PINS = [17, 22, 23, 24]

def init():
    gpio.setmode(gpio.BCM)
    for pin in MOTOR_PINS:
        gpio.setup(pin, gpio.OUT)

def stop():
    for pin in MOTOR_PINS:
        gpio.output(pin, False)

def right(duration=1.5):
    gpio.output(17, False)
    gpio.output(22, True)
    gpio.output(23, False)
    gpio.output(24, True)
    time.sleep(duration)
    stop()

init()
print("Testing right turn for 1.5 seconds...")
right(1.5)
stop()
gpio.cleanup()