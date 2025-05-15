import RPi.GPIO as gpio
import time
import cv2
from picamera2 import Picamera2

# Motor pins
MOTOR_PINS = [17, 22, 23, 24]
TRIG = 6
ECHO = 5

def init():
    gpio.setmode(gpio.BCM)
    for pin in MOTOR_PINS:
        gpio.setup(pin, gpio.OUT)
    gpio.setup(TRIG, gpio.OUT)
    gpio.setup(ECHO, gpio.IN)

def forward(duration=0.2):
    gpio.output(17, False)
    gpio.output(22, True)
    gpio.output(23, True)
    gpio.output(24, False)
    time.sleep(duration)
    stop()
    time.sleep(0.05)

def backward(duration=0.2):
    gpio.output(17, True)
    gpio.output(22, False)
    gpio.output(23, False)
    gpio.output(24, True)
    time.sleep(duration)
    stop()
    time.sleep(0.05)

def left(duration=0.2):
    gpio.output(17, True)
    gpio.output(22, False)
    gpio.output(23, True)
    gpio.output(24, False)
    time.sleep(duration)
    stop()
    time.sleep(0.05)

def right(duration=0.2):
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
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate = 16
picam2.configure("preview")
picam2.start()
time.sleep(0.1)

hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

print("Starting people and obstacle tracking.")
print("Press 'm' to toggle TEST MODE (manual keyboard control).")
print("In test mode: Arrow keys to move, 's' to stop, 'q' to quit.")

init()

moving = False
manual_mode = False

try:
    while True:
        image_rgb = picam2.capture_array()
        image_rgb = cv2.rotate(image_rgb, cv2.ROTATE_180)

        # Detect people (shoes/legs) on RGB image
        (rects, weights) = hog.detectMultiScale(
            image_rgb,
            winStride=(4, 4),
            padding=(8, 8),
            scale=1.01
        )

        # Filter detections by minimum size (ignore small boxes)
        min_width, min_height = 40, 80
        filtered_rects = []
        for (x, y, w, h) in rects:
            if w >= min_width and h >= min_height:
                filtered_rects.append((x, y, w, h))

        person_detected = False
        person_center_x = None
        person_box = None
        largest_area = 0
        image_draw = image_rgb.copy()
        for (x, y, w, h) in filtered_rects:
            cv2.rectangle(image_draw, (x, y), (x + w, y + h), (0, 255, 0), 2)
            area = w * h
            center_x = x + w // 2
            # Center region for following, pick largest
            if 220 < center_x < 420 and area > largest_area:
                person_detected = True
                person_center_x = center_x
                person_box = (x, y, w, h)
                largest_area = area

        distance = measure_distance()
        cv2.putText(image_draw, f"Distance: {distance}cm", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)

        # Convert to BGR for OpenCV display
        image_bgr = cv2.cvtColor(image_draw, cv2.COLOR_RGB2BGR)
        cv2.imshow("People & Obstacle Tracking", image_bgr)

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
            print(f"Detections: {len(filtered_rects)}")
            # Obstacle detected close: stop
            if distance <= 30:
                print("Obstacle detected! Stopping.")
                stop()
                moving = False
            # Person detected in center: follow and keep distance
            elif person_detected:
                print("Person detected in center!")
                if distance > 80:
                    print("Too far, moving forward to follow.")
                    forward()
                    moving = True
                elif distance < 30:
                    print("Too close, stopping.")
                    stop()
                    moving = False
                else:
                    print("Keeping distance.")
                    stop()
                    moving = False
            else:
                # No person, no obstacle: move forward
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