from time import sleep

import RPi.GPIO as GPIO

# GPIO setup
GPIO.setmode(GPIO.BCM)

# Motor 1 pins
motor1_pins = [17, 27, 4]
GPIO.setup(motor1_pins, GPIO.OUT)

# Motor 2 pins
motor2_pins = [ 6]
GPIO.setup(motor2_pins, GPIO.OUT)

# PWM pin
pwm_pin = 13
GPIO.setup(pwm_pin, GPIO.OUT)

# Initialize PWM
pwm = GPIO.PWM(pwm_pin, 100)  # 100 Hz frequency
pwm.start(0)  # Start with 0% duty cycle

try:
    # Example: Turn on Motor 1
    GPIO.output(motor1_pins, (GPIO.HIGH, GPIO.LOW, GPIO.LOW))
    pwm.ChangeDutyCycle(50)  # Set PWM to 50% duty cycle
    sleep(2)

    # Example: Turn on Motor 2
    GPIO.output(motor2_pins, (GPIO.HIGH, GPIO.LOW))
    pwm.ChangeDutyCycle(75)  # Set PWM to 75% duty cycle
    sleep(2)

finally:
    # Cleanup
    pwm.stop()
    GPIO.cleanup()
