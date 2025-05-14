import RPi.GPIO as gpio
import time
import cv2
from picamera2 import Picamera2

# Motor pins
MOTOR_PINS = [17, 22, 23, 24]

# Ultrasonic sensor pins (GPIO numbers)
TRIG = 6   # GPIO 5 (physical pin 29)
ECHO = 5   # GPIO 6 (physical pin 31)

def init():
    gpio.setmode(gpio.BCM)
    for pin in MOTOR_PINS:
        gpio.setup(pin, gpio.OUT)
    gpio.setup(TRIG, gpio.OUT)
    gpio.setup(ECHO, gpio.IN)

def forward(duration=0.05):
    gpio.output(17, False)
    gpio.output(22, True)
    gpio.output(23, True)
    gpio.output(24, False)
    time.sleep(duration)  # Short pulse ON
    stop()
    time.sleep(0.05)  # Short pause OFF

def backward(duration=0.05):
    gpio.output(17, True)
    gpio.output(22, False)
    gpio.output(23, False)
    gpio.output(24, True)
    time.sleep(duration)
    stop()
    time.sleep(0.05)

def left(duration=0.05):
    gpio.output(17, True)
    gpio.output(22, False)
    gpio.output(23, True)
    gpio.output(24, False)
    time.sleep(duration)
    stop()
    time.sleep(0.05)

def right(duration=0.05):
    gpio.output(17, False)
    gpio.output(22, True)
    gpio.output(23, False)
    gpio.output(24, True)
    time.sleep(duration)
    stop()
    time.sleep(0.05)

def stop():
    gpio.output(17, False)
    gpio.output(22, False)
    gpio.output(23, False)
    gpio.output(24, False)

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
            return 0

    pulse_end = time.time()
    timeout = pulse_end + 0.04

    while gpio.input(ECHO) == 1:
        pulse_end = time.time()
        if pulse_end > timeout:
            print("Timeout waiting for ECHO to go low")
            return 0

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance, 2)
    return distance

# Camera and people detection setup using picamera2
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"  # Use RGB formats
picam2.preview_configuration.controls.FrameRate = 16
picam2.configure("preview")
picam2.start()
time.sleep(0.1)

print("Showing camera preview for 3 seconds. Adjust focus if needed.")
for _ in range(30):
    frame = picam2.capture_array()
    frame = cv2.rotate(frame, cv2.ROTATE_180)  # Rotate preview
    cv2.imshow("Camera Preview", frame)  # Show frame directly, no color conversion
    cv2.waitKey(100)
cv2.destroyWindow("Camera Preview")

hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

print("Starting people and obstacle tracking.")
print("Press 'm' to toggle TEST MODE (manual keyboard control).")
print("In test mode: Arrow keys to move, 's' to stop, 'q' to quit.")

init()  # Initialize GPIOs once at the start

moving = False  # Track if robot is moving
manual_mode = False  # Start in automatic mode

try:
    while True:
        image = picam2.capture_array()
        image = cv2.rotate(image, cv2.ROTATE_180)  # Rotate camera image
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)  # Convert for OpenCV

        distance = measure_distance()
        cv2.putText(image, f"Distance: {distance}cm", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)

        cv2.imshow("People & Obstacle Tracking", image)
        key = cv2.waitKey(1) & 0xFF

        # Press 'm' to toggle manual mode ON/OFF
        if key == ord('m'):
            manual_mode = not manual_mode
            print("TEST MODE ON (manual control)" if manual_mode else "TEST MODE OFF (auto mode)")
            stop()

        if manual_mode:
            if key == ord('q'):
                break
            elif key == 82:  # Up arrow
                print("Manual: Forward")
                forward()
            elif key == 84:  # Down arrow
                print("Manual: Backward")
                backward()
            elif key == 81:  # Left arrow
                print("Manual: Left")
                left()
            elif key == 83:  # Right arrow
                print("Manual: Right")
                right()
            elif key == ord('s'):
                print("Manual: Stop")
                stop()
        else:
            (rects, weights) = hog.detectMultiScale(
                image,
                winStride=(8, 8),
                padding=(16, 16),
                scale=1.05
            )

            print(f"Detections: {len(rects)}")  # Debug: print number of detections

            person_detected = False
            for (x, y, w, h) in rects:
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                center_x = x + w // 2
                if 220 < center_x < 420:
                    person_detected = True

            # Motor logic: only change state if needed
            if person_detected:
                if moving:
                    print("Person detected! Stopping.")
                    stop()
                    moving = False
            elif distance <= 30:
                print("Obstacle detected! Going backward.")
                backward(0.5)
                stop()
                time.sleep(0.1)
                # Check left
                print("Checking left...")
                left(0.5)
                stop()
                time.sleep(0.1)
                left_distance = measure_distance()
                if left_distance > 30:
                    print("Left is clear, turning left.")
                    left(0.7)  # <-- More power/time for actual turn
                    stop()
                    moving = False
                else:
                    print("Left blocked, checking right...")
                    # Go back to original direction
                    right(1.0)
                    stop()
                    time.sleep(0.1)
                    right_distance = measure_distance()
                    if right_distance > 30:
                        print("Right is clear, turning right.")
                        right(0.7)  # <-- More power/time for actual turn
                        stop()
                        moving = False
                    else:
                        print("Both sides blocked, stopping.")
                        stop()
                        moving = False
            else:
                if not moving:
                    print("Path clear, moving forward")
                    forward()
                    moving = True

        if key == ord("q"):
            break

finally:
    stop()
    gpio.cleanup()
    cv2.destroyAllWindows()
    picam2.close()
    print("Program ended")