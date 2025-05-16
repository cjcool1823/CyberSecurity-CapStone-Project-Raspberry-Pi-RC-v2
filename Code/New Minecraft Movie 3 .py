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

# --- continuous‐drive primitives (no sleep) ---
def forward_start():
    gpio.output(17, False)
    gpio.output(22, True)
    gpio.output(23, True)
    gpio.output(24, False)

def backward_start():
    gpio.output(17, True)
    gpio.output(22, False)
    gpio.output(23, False)
    gpio.output(24, True)

def left_start():
    gpio.output(17, True)
    gpio.output(22, False)
    gpio.output(23, True)
    gpio.output(24, False)

def right_start():
    gpio.output(17, False)
    gpio.output(22, True)
    gpio.output(23, False)
    gpio.output(24, True)

def stop():
    for pin in MOTOR_PINS:
        gpio.output(pin, False)

# --- manual test‐mode pulses ---
def forward(duration=0.5):
    forward_start()
    time.sleep(duration)
    stop()
    time.sleep(0.05)

def backward(duration=0.5):
    backward_start()
    time.sleep(duration)
    stop()
    time.sleep(0.05)

def left(duration=0.5):
    left_start()
    time.sleep(duration)
    stop()
    time.sleep(0.05)

def right(duration=0.5):
    right_start()
    time.sleep(duration)
    stop()
    time.sleep(0.05)

def measure_distance():
    gpio.output(TRIG, False)
    time.sleep(0.05)
    gpio.output(TRIG, True)
    time.sleep(0.0001)
    gpio.output(TRIG, False)

    start = time.time()
    timeout = start + 0.04
    while gpio.input(ECHO) == 0:
        start = time.time()
        if start > timeout:
            return 999
    end = time.time()
    timeout = end + 0.04
    while gpio.input(ECHO) == 1:
        end = time.time()
        if end > timeout:
            return 999

    dist = (end - start) * 17150
    return round(dist, 2)

# set up camera + HOG
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate = 16
picam2.configure("preview")
picam2.start()
time.sleep(0.1)

hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

init()
moving = False
manual_mode = False

try:
    while True:
        frame_rgb = picam2.capture_array()
        frame_rgb = cv2.rotate(frame_rgb, cv2.ROTATE_180)

        # detect legs/shoes
        rects, _ = hog.detectMultiScale(
            frame_rgb, winStride=(4,4), padding=(8,8), scale=1.01
        )
        # filter small boxes
        filtered = [(x,y,w,h) for (x,y,w,h) in rects if w>=30 and h>=60]

        # pick largest center box (not used in auto-logic below)
        person_detected = False
        best_area = 0
        for x,y,w,h in filtered:
            cx = x + w//2
            area = w*h
            if 220 < cx < 420 and area > best_area:
                person_detected = True
                best_area = area

        dist = measure_distance()

        # draw rectangles and distance on frame
        disp = frame_rgb.copy()
        for x,y,w,h in filtered:
            cv2.rectangle(disp, (x,y), (x+w,y+h), (0,255,0), 2)
        cv2.putText(disp, f"D={dist}cm", (10,20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
        disp_bgr = cv2.cvtColor(disp, cv2.COLOR_RGB2BGR)
        cv2.imshow("Tracking", disp_bgr)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('m'):
            manual_mode = not manual_mode
            print("Manual mode:", manual_mode)
            stop()
            moving = False

        if manual_mode:
            if key == 82: forward()
            if key == 84: backward()
            if key == 81: left()
            if key == 83: right()
            if key == ord('s'): stop()
            if key == ord('q'): break
        else:
            # AUTO MODE: drive if path clear; stop if obstacle
            if dist <= 30:
                stop()
                moving = False
            else:
                forward_start()
                moving = True

            if key == ord('q'):
                break

finally:
    stop()
    gpio.cleanup()
    cv2.destroyAllWindows()
    picam2.close()
    print("Ended")