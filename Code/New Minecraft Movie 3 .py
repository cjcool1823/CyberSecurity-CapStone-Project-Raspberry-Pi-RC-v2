import os, sys
# suppress libpng “iCCP: known incorrect sRGB profile” warnings
sys.stderr = open(os.devnull, 'w')

import RPi.GPIO as gpio
import time
import cv2
import mediapipe as mp
from picamera2 import Picamera2
import traceback

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

# continuous‐drive primitives (no sleep)
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

# manual‐mode pulses
def forward(duration=0.5):
    forward_start(); time.sleep(duration); stop(); time.sleep(0.05)

def backward(duration=0.5):
    backward_start(); time.sleep(duration); stop(); time.sleep(0.05)

def left(duration=0.5):
    left_start(); time.sleep(duration); stop(); time.sleep(0.05)

def right(duration=0.5):
    right_start(); time.sleep(duration); stop(); time.sleep(0.05)

def measure_distance():
    gpio.output(TRIG, False); time.sleep(0.01)
    gpio.output(TRIG, True);  time.sleep(0.0001)
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

    duration = end - start
    return round(duration * 17150, 2)

# mediapipe hands
mp_hands   = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
hands      = mp_hands.Hands(
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5)

def main():
    init()

    # Picamera2 setup
    picam2 = Picamera2()
    picam2.preview_configuration.main.size    = (640, 480)
    picam2.preview_configuration.main.format  = "RGB888"
    picam2.preview_configuration.controls.FrameRate = 30
    picam2.configure("preview")
    picam2.start()
    time.sleep(0.05)

    # HOG people-detector (legs/shoes)
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    moving = False
    manual_mode = False

    try:
        while True:
            # 1) capture and rotate
            frame = picam2.capture_array()
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            disp  = frame.copy()

            # 2) HOG detect legs/shoes
            rects, _ = hog.detectMultiScale(
                frame, winStride=(4,4), padding=(8,8), scale=1.01
            )
            filtered = [(x,y,w,h) for x,y,w,h in rects if w>=30 and h>=60]

            person = False
            best_box = None
            best_area = 0
            for x,y,w,h in filtered:
                area = w*h
                cx   = x + w//2
                if 220 < cx < 420 and area > best_area:
                    person    = True
                    best_area = area
                    best_box  = (x,y,w,h)
                cv2.rectangle(disp, (x,y), (x+w,y+h), (0,255,0), 2)

            # 3) shirt region (upper 40%)
            if best_box:
                x,y,w,h = best_box
                sy = y + int(0.1*h)
                sh = int(0.4*h)
                cv2.rectangle(disp, (x,sy), (x+w,sy+sh), (255,0,0), 2)

            # 4) hand landmarks
            results = hands.process(frame)
            hand_detected = bool(results.multi_hand_landmarks)
            if hand_detected:
                for lm in results.multi_hand_landmarks:
                    mp_drawing.draw_landmarks(
                        disp, lm, mp_hands.HAND_CONNECTIONS)

            # 5) distance
            dist = measure_distance()
            cv2.putText(disp, f"D={dist}cm", (10,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

            # 6) display
            cv2.imshow("Tracking",
                       cv2.cvtColor(disp, cv2.COLOR_RGB2BGR))

            key = cv2.waitKey(1) & 0xFF
            if key == ord('m'):
                manual_mode = not manual_mode
                stop(); moving = False
                print("Manual mode:", manual_mode)
            if key == ord('q'):
                break

            # 7) control logic
            if manual_mode:
                if key == 82:    # Up
                    forward()
                elif key == 84:  # Down
                    backward()
                elif key == 81:  # Left
                    left()
                elif key == 83:  # Right
                    right()
                elif key == ord('s'):
                    stop()
                continue
            else:
                # AUTO: move only when no obstacle, leg/box centered & hand present
                if dist > 30 and person and hand_detected:
                    if not moving:
                        forward_start()
                        moving = True
                else:
                    if moving:
                        stop()
                        moving = False

    except Exception as e:
        # print any errors
        print("Error:", e)
        traceback.print_exc()
    finally:
        stop()
        hands.close()
        gpio.cleanup()
        cv2.destroyAllWindows()
        picam2.close()
        print("Program ended")

if __name__ == "__main__":
    main()
