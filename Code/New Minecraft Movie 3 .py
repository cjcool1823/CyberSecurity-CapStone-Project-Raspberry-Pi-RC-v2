import os, sys
# suppress libpng warnings
sys.stderr = open(os.devnull, 'w')

import RPi.GPIO as gpio
import time
import cv2
import mediapipe as mp
from picamera2 import Picamera2

# Motor pins
MOTOR_PINS = [17, 22, 23, 24]
TRIG = 6
ECHO = 5

def init():
    gpio.setmode(gpio.BCM)
    for p in MOTOR_PINS:
        gpio.setup(p, gpio.OUT)
    gpio.setup(TRIG, gpio.OUT)
    gpio.setup(ECHO, gpio.IN)

# continuous‐drive (no sleep)
def forward_start():
    gpio.output(17, False); gpio.output(22, True)
    gpio.output(23, True);  gpio.output(24, False)

def backward_start():
    gpio.output(17, True);  gpio.output(22, False)
    gpio.output(23, False); gpio.output(24, True)

def left_start():
    gpio.output(17, True);  gpio.output(22, False)
    gpio.output(23, True);  gpio.output(24, False)

def right_start():
    gpio.output(17, False); gpio.output(22, True)
    gpio.output(23, False); gpio.output(24, True)

def stop():
    for p in MOTOR_PINS:
        gpio.output(p, False)

# manual pulses
def forward(duration=0.5):
    forward_start(); time.sleep(duration); stop()

def backward(duration=0.5):
    backward_start(); time.sleep(duration); stop()

def left(duration=0.5):
    left_start(); time.sleep(duration); stop()

def right(duration=0.5):
    right_start(); time.sleep(duration); stop()

def measure_distance():
    gpio.output(TRIG, False); time.sleep(0.01)
    gpio.output(TRIG, True); time.sleep(0.00005)
    gpio.output(TRIG, False)
    start = time.time(); timeout = start + 0.02
    while gpio.input(ECHO) == 0:
        start = time.time()
        if start > timeout: return 999
    end = time.time(); timeout = end + 0.02
    while gpio.input(ECHO) == 1:
        end = time.time()
        if end > timeout: return 999
    return round((end - start) * 17150, 2)

# mediapipe hand detector
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
hands = mp_hands.Hands(
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5)

# camera setup
picam2 = Picamera2()
picam2.preview_configuration.main.size   = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate = 30
picam2.configure("preview"); picam2.start()
time.sleep(0.05)

# HOG people detector
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

init()
moving = False
manual_mode = False

try:
    while True:
        # capture RGB frame
        frame_rgb = picam2.capture_array()
        frame_rgb = cv2.rotate(frame_rgb, cv2.ROTATE_180)
        frame_disp = frame_rgb.copy()

        # ---------- detect legs/shoes (HOG) ----------
        rects, _ = hog.detectMultiScale(
            frame_rgb, winStride=(4,4), padding=(8,8), scale=1.01)
        filtered = [(x,y,w,h) for x,y,w,h in rects if w>=30 and h>=60]

        person = False; best_area = 0; best_box = None
        for x,y,w,h in filtered:
            cx = x + w//2; area = w*h
            if 220 < cx < 420 and area > best_area:
                person = True; best_area = area; best_box = (x,y,w,h)
            cv2.rectangle(frame_disp, (x,y), (x+w,y+h), (0,255,0), 2)

        # ---------- detect shirt region ----------
        if best_box:
            x,y,w,h = best_box
            # shirt approximated as upper 40% of the person box
            sy = y + int(0.1*h)
            sh = int(0.4*h)
            cv2.rectangle(frame_disp,
                (x, sy), (x+w, sy+sh), (255,0,0), 2)

        # ---------- detect hands (MediaPipe) ----------
        results = hands.process(frame_rgb)
        if results.multi_hand_landmarks:
            for lm in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(
                    frame_disp, lm, mp_hands.HAND_CONNECTIONS)

        # ---------- distance measurement ----------
        dist = measure_distance()
        cv2.putText(frame_disp, f"D={dist}cm", (10,30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

        # display (convert RGB→BGR)
        disp_bgr = cv2.cvtColor(frame_disp, cv2.COLOR_RGB2BGR)
        cv2.imshow("Tracking", disp_bgr)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('m'):
            manual_mode = not manual_mode
            stop(); moving = False
            print("Manual mode:", manual_mode)

        if manual_mode:
            if key == 82: forward()
            if key == 84: backward()
            if key == 81: left()
            if key == 83: right()
            if key == ord('s'): stop()
            if key == ord('q'): break
        else:
            # AUTO MODE: follow when see shirt & hand and no obstacle
            if dist > 30 and person and results.multi_hand_landmarks:
                forward_start(); moving = True
            else:
                if moving:
                    stop(); moving = False
            if key == ord('q'):
                break

finally:
    stop()
    hands.close()
    gpio.cleanup()
    cv2.destroyAllWindows()
    picam2.close()
    print("Ended")
