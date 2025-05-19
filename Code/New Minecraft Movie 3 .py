import os, sys
# suppress libpng “iCCP: known incorrect sRGB profile” warnings
sys.stderr = open(os.devnull, 'w')

import RPi.GPIO as gpio
import time
import cv2
from picamera2 import Picamera2

# Motor pins (L298N + Dagu DG01D chassis)
# Motor A (left): IN1=17, IN2=22
# Motor B (right): IN3=23, IN4=24
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
    # both motors forward
    gpio.output(17, False); gpio.output(22, True)
    gpio.output(23, True);  gpio.output(24, False)

def backward_start():
    # both motors backward
    gpio.output(17, True);  gpio.output(22, False)
    gpio.output(23, False); gpio.output(24, True)

def pivot_left_start():
    # left motor back, right motor forward → turn left in place
    gpio.output(17, True);  gpio.output(22, False)
    gpio.output(23, True);  gpio.output(24, False)

def pivot_right_start():
    # left motor forward, right motor back → turn right in place
    gpio.output(17, False); gpio.output(22, True)
    gpio.output(23, False); gpio.output(24, True)

def stop():
    for p in MOTOR_PINS:
        gpio.output(p, False)

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
    return round((end - start) * 17150, 2)

def main():
    init()

    # camera + HOG setup
    picam2 = Picamera2()
    picam2.preview_configuration.main.size    = (640, 480)
    picam2.preview_configuration.main.format  = "RGB888"
    picam2.preview_configuration.controls.FrameRate = 30
    picam2.configure("preview")
    picam2.start()
    time.sleep(0.05)

    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    moving = False

    try:
        while True:
            frame = picam2.capture_array()
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            disp  = frame.copy()

            # detect legs/shoes
            rects, _ = hog.detectMultiScale(
                frame, winStride=(4,4), padding=(8,8), scale=1.01
            )
            filtered = [(x,y,w,h) for x,y,w,h in rects if w>=30 and h>=60]

            # pick largest centered human box
            person = False; best_area = 0
            for x,y,w,h in filtered:
                cx = x + w//2; area = w*h
                cv2.rectangle(disp, (x,y), (x+w,y+h), (0,255,0), 2)
                if 220 < cx < 420 and area > best_area:
                    person = True; best_area = area

            # distance
            dist = measure_distance()
            cv2.putText(disp, f"D={dist}cm", (10,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)

            cv2.imshow("Tracking", cv2.cvtColor(disp, cv2.COLOR_RGB2BGR))
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

            # AUTO logic:
            #  - obstacle close → pivot left to avoid
            #  - person centered & clear → forward
            #  - else → pivot right to search
            if dist <= 30:
                stop(); moving = False
                pivot_left_start()
                moving = True
            elif person and dist > 30:
                forward_start(); moving = True
            else:
                stop(); moving = False
                pivot_right_start()
                moving = True

    finally:
        stop()
        gpio.cleanup()
        cv2.destroyAllWindows()
        picam2.close()
        print("Program ended")

if __name__ == "__main__":
    main()
