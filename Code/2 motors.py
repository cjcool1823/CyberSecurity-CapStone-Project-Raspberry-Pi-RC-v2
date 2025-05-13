import time
import RPi.GPIO as GPIO

# Pin configuration for L298N
IN1 = 11  # Motor 1 IN1
IN2 = 15  # Motor 1 IN2
IN3 = 18  # Motor 2 IN3
IN4 = 16  # Motor 2 IN4

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

def motor1_forward():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)

def motor1_backward():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)

def motor2_forward():
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

def motor2_backward():
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

def motors_stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.LOW)

try:
    # Example usage
    print("Motor 1 and Motor 2 forward")
    motor1_forward()
    motor2_forward()
    time.sleep(2)

    print("Motor 1 and Motor 2 backward")
    motor1_backward()
    motor2_backward()
    time.sleep(2)

    print("Motors stop")
    motors_stop()

finally:
    GPIO.cleanup()  # Cleanup GPIO settings
    print("Program ended")