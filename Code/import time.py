import time

from gpiozero import M1213otor

# Pin configuration
IN1 = 2  # GPIO pin 2
IN2 = 3  # GPIO pin 3

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)

def motor_forward():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)

def motor_backward():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)

def motor_stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)

try:
    print("Motor running forward")
    motor_forward()
    time.sleep(5)  # Run motor forward for 5 seconds

    print("Motor running backward")
    motor_backward()
    time.sleep(5)  # Run motor backward for 5 seconds

    print("Stopping motor")
    motor_stop()

finally:
    GPIO.cleanup()  # Clean up GPIO settings
