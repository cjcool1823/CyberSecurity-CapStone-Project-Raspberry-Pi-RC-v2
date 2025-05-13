import RPi.GPIO as GPIO
import time
import cv2
import numpy as np

# Pin configuration for motors
motor_pin1 = 2  # Shared control pin 1 for all motors
motor_pin2 = 3  # Shared control pin 2 for all motors

# Pin configuration for ultrasonic sensor
TRIG = 23  # Trigger pin
ECHO = 24  # Echo pin

# GPIO setup
GPIO.setmode(GPIO.BCM)  # Use BCM pin numbering
GPIO.setup(motor_pin1, GPIO.OUT)
GPIO.setup(motor_pin2, GPIO.OUT)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

def move_motors(forward, duration=None):
    """Control all motors simultaneously."""
    if forward:
        GPIO.output(motor_pin1, GPIO.HIGH)
        GPIO.output(motor_pin2, GPIO.LOW)
    else:
        GPIO.output(motor_pin1, GPIO.LOW)
        GPIO.output(motor_pin2, GPIO.HIGH)
    
    if duration:
        time.sleep(duration)
        GPIO.output(motor_pin1, GPIO.LOW)
        GPIO.output(motor_pin2, GPIO.LOW)

def stop_motors():
    """Stop all motors."""
    GPIO.output(motor_pin1, GPIO.LOW)
    GPIO.output(motor_pin2, GPIO.LOW)

def get_distance():
    """Measure distance using the ultrasonic sensor."""
    GPIO.output(TRIG, False)
    time.sleep(0.1)

    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()

    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Convert to cm
    return round(distance, 2)

def detect_person(frame):
    """Detect a person in the frame using Haar cascades."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    person_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_fullbody.xml')
    persons = person_cascade.detectMultiScale(gray, 1.1, 4)

    if len(persons) > 0:
        # Get the largest detected person (closest to the camera)
        x, y, w, h = max(persons, key=lambda rect: rect[2] * rect[3])
        return (x, y, w, h)
    return None

try:
    print("Waiting for a person to trigger the robot...")
    cap = cv2.VideoCapture(0)  # Open the Raspberry Pi Camera

    while True:
        # Read a frame from the camera
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame")
            break

        # Detect a person in the frame
        person = detect_person(frame)
        if person:
            print("Person detected! Starting the robot...")
            break  # Exit the waiting loop and start the robot

    # Main robot control loop
    while True:
        # Get distance from ultrasonic sensor
        distance = get_distance()
        print(f"Distance: {distance} cm")

        # Read a frame from the camera
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame")
            break

        # Detect a person in the frame
        person = detect_person(frame)
        if person:
            x, y, w, h = person
            print(f"Person detected at x={x}, y={y}, width={w}, height={h}")

            # Draw a rectangle around the detected person
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

            # Determine movement based on person's position
            center_x = x + w // 2
            frame_center = frame.shape[1] // 2

            if center_x < frame_center - 50:  # Person is to the left
                print("Turning left...")
                move_motors(False, 0.2)
            elif center_x > frame_center + 50:  # Person is to the right
                print("Turning right...")
                move_motors(True, 0.2)
            else:  # Person is centered
                if distance > 50:  # Too far, move forward
                    print("Moving forward...")
                    move_motors(True)
                elif distance < 30:  # Too close, stop
                    print("Stopping...")
                    stop_motors()
        else:
            print("No person detected. Stopping...")
            stop_motors()

        # Display the frame (for debugging purposes)
        cv2.imshow("Frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Stopping robot...")
finally:
    cap.release()
    cv2.destroyAllWindows()
    GPIO.cleanup()
    print("GPIO cleanup done.")