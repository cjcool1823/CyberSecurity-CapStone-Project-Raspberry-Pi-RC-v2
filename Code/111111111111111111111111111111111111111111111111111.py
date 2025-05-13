import time

import RPi.GPIO as GPIO

# Pin configuration
INT1 = 14  # GPIO 14, Pin 8
INT2 = 15  # GPIO 15, Pin 10

# Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(INT1, GPIO.OUT)
GPIO.setup(INT2, GPIO.OUT)

def motor_forward():
    GPIO.output(INT1, GPIO.HIGH)
    GPIO.output(INT2, GPIO.LOW)

def motor_backward():
    GPIO.output(INT1, GPIO.LOW)
    GPIO.output(INT2, GPIO.HIGH)

def motor_stop():
    GPIO.output(INT1, GPIO.LOW)
    GPIO.output(INT2, GPIO.LOW)

try:
    while True:
        print("Motor Forward")
        motor_forward()
        time.sleep(2)

        print("Motor Backward")
        motor_backward()
        time.sleep(2)

        print("Motor Stop")
        motor_stop()
        time.sleep(2)

except KeyboardInterrupt:
    print("Exiting program")

finally:
    GPIO.cleanup()