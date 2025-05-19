import os, sys
# suppress libpng “iCCP: known incorrect sRGB profile” warnings
sys.stderr = open(os.devnull, 'w')

import RPi.GPIO as gpio
import time
import cv2
from picamera2 import Picamera2

# Motor pins (L298N + Dagu DG01D chassis)
MOTOR_PINS = [17, 22, 23, 24]
TRIG = 6
ECHO = 5

def init():
    gpio.setmode(gpio.BCM)
    for p in MOTOR_PINS:
        gpio.setup(p, gpio.OUT)
    gpio.setup(TRIG, gpio.OUT)
    gpio.setup(ECHO, gpio.IN)

def forward_start():
    gpio.output(17, False); gpio.output(22, True)
    gpio.output(23, True);  gpio.output(24, False)

def pivot_left_start():
    gpio.output(17, True);  gpio.output(22, False)
    gpio.output(23, True);  gpio.output(24, False)

def pivot_right_start():
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
    while gpio.input(ECHO) == 0 and time.time() < timeout:
        pass
    pulse_start = time.time()
    timeout = pulse_start + 0.04
    while gpio.input(ECHO) == 1 and time.time() < timeout:
        pass
    pulse_end = time.time()
    return round((pulse_end - pulse_start) * 17150, 2)

def main():
    init()

    # camera setup at lower resolution & fps to reduce CPU
    picam2 = Picamera2()
    picam2.preview_configuration.main.size    = (320, 240)
    picam2.preview_configuration.main.format  = "RGB888"
    picam2.preview_configuration.controls.FrameRate = 10
    picam2.configure("preview")
    picam2.start()
    time.sleep(0.1)

    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    moving = False
    frame_count = 0
    person = False

    try:
        while True:
            frame = picam2.capture_array()
            # rotate & optionally resize to speed up display
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            disp  = frame.copy()

            # convert to gray once
            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

            # run HOG only every 4 frames
            if frame_count % 4 == 0:
                rects, _ = hog.detectMultiScale(
                    gray,
                    winStride=(8,8),
                    padding=(8,8),
                    scale=1.05
                )
                best_area = 0
                person = False
                for x,y,w,h in rects:
                    if h < 80 or w < 20:
                        continue
                    area = w*h; cx = x + w//2
                    if 120 < cx < 200 and area > best_area:
                        best_area = area
                        person = True
                    cv2.rectangle(disp, (x,y), (x+w,y+h), (0,255,0), 1)

            dist = measure_distance()
            cv2.putText(disp, f"D={dist:.1f}cm", (5,15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)

            cv2.imshow("Tracking", cv2.cvtColor(disp, cv2.COLOR_RGB2BGR))
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # AUTO logic (obstacle → left pivot, follow → forward, else → right pivot)
            if dist <= 30:
                stop(); moving = False
                pivot_left_start(); moving = True
            elif person:
                if not moving:
                    forward_start(); moving = True
            else:
                stop(); moving = False
                pivot_right_start(); moving = True

            frame_count += 1

    finally:
        stop()
        gpio.cleanup()
        cv2.destroyAllWindows()
        picam2.close()
        print("Program ended")

if __name__ == "__main__":
    main()
