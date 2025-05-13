import time
from gpiozero import Motor

# Pin configuration
IN1 = 2  # GPIO pin 2
IN2 = 3  # GPIO pin 3

# Initialize motor
motor = Motor(forward=IN1, backward=IN2)

try:
    print("Motor running forward")
    motor.forward()  # Run motor forward
    time.sleep(5)  # Run motor forward for 5 seconds

    print("Motor running backward")
    motor.backward()  # Run motor backward
    time.sleep(5)  # Run motor backward for 5 seconds

    print("Stopping motor")
    motor.stop()  # Stop the motor

finally:
    print("Exiting program")