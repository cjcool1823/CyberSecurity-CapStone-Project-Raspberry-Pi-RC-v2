import RPi.GPIO as GPIO
import time

# Setup GPIO mode
GPIO.setmode(GPIO.BCM)

# Motor 1 pins
motor1_pins = [17, 27, 4]
# Motor 2 pins
motor2_pins = [5, 6]

# Setup motor 1 pins as output
for pin in motor1_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

# Setup motor 2 pins as output
for pin in motor2_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

try:
    # Example: Turn on motor 1
    GPIO.output(motor1_pins[0], GPIO.HIGH)
    time.sleep(2)
    GPIO.output(motor1_pins[0], GPIO.LOW)

    # Example: Turn on motor 2
    GPIO.output(motor2_pins[0], GPIO.HIGH)
    time.sleep(2)
    GPIO.output(motor2_pins[0], GPIO.LOW)

finally:
    # Cleanup GPIO
    GPIO.cleanup()