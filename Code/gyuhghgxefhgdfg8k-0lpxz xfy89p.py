import time
from gpiozero import Motor

# Define motors with their respective GPIO pins
motor1 = Motor(forward=2, backward=3)  # Motor 1
motor2 = Motor(forward=10, backward=12)  # Motor 2

try:
    # Example usage
    print("Motor 1 and Motor 2 forward")
    motor1.forward()
    motor2.forward()
    time.sleep(2)

    print("Motor 1 and Motor 2 backward")
    motor1.backward()
    motor2.backward()
    time.sleep(2)

    print("Motors stop")
    motor1.stop()
    motor2.stop()

finally:
    print("Program ended")
