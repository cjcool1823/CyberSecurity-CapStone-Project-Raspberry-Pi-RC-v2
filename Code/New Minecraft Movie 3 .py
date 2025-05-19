import os, sys
# temporarily remove stderr redirection to see errors
# sys.stderr = open(os.devnull, 'w')

import RPi.GPIO as gpio
import time
import cv2
from picamera2 import Picamera2
import traceback

# Motor pins (L298N + Dagu DG01D)
MOTOR_PINS = [17, 22, 23, 24]
TRIG = 6
ECHO = 5

def init_gpio():
    gpio.setmode(gpio.BCM)
    for p in MOTOR_PINS:
        gpio.setup(p, gpio.OUT)
    gpio.setup(TRIG, gpio.OUT)
    gpio.setup(ECHO, gpio.IN)

def forward_start():
    gpio.output(17, False); gpio.output(22, True)
    gpio.output(23, True);  gpio.output(24, False)

def stop():
    for p in MOTOR_PINS:
        gpio.output(p, False)

def measure_distance():
    gpio.output(TRIG, False)
    time.sleep(0.01)
    gpio.output(TRIG, True)
    time.sleep(0.0001)
    gpio.output(TRIG, False)

    start = time.time()
    while gpio.input(ECHO) == 0 and time.time() - start < 0.04:
        pass
    t0 = time.time()
    while gpio.input(ECHO) == 1 and time.time() - t0 < 0.04:
        pass
    duration = time.time() - t0
    return round(duration * 17150, 2)

def main():
    init_gpio()

    try:
        # Picamera2 low-res config to reduce CPU
        picam2 = Picamera2()
        picam2.preview_configuration.main.size    = (320, 240)
        picam2.preview_configuration.main.format  = "RGB888"
        picam2.preview_configuration.controls.FrameRate = 10
        picam2.configure("preview")
        picam2.start()
        time.sleep(0.1)

        # HOG people detector
        hog = cv2.HOGDescriptor()
        hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        moving = False
        frame_count = 0

        while True:
            frame = picam2.capture_array()
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            disp  = frame.copy()

            gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

            person = False
            # run HOG every 4 frames
            if frame_count % 4 == 0:
                rects, _ = hog.detectMultiScale(
                    gray, winStride=(8,8), padding=(8,8), scale=1.05
                )
                best_area = 0
                for x,y,w,h in rects:
                    if w < 20 or h < 60: 
                        continue
                    area = w*h; cx = x + w//2
                    cv2.rectangle(disp, (x,y), (x+w,y+h), (0,255,0), 1)
                    if 120 < cx < 200 and area > best_area:
                        best_area = area
                        person = True

            dist = measure_distance()
            cv2.putText(disp, f"D={dist}cm", (5,15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)

            cv2.imshow("Tracking", cv2.cvtColor(disp, cv2.COLOR_RGB2BGR))
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

            # AUTO only: stop on obstacle, else follow person, else stop
            if dist <= 30:
                if moving:
                    stop(); moving = False
            elif person:
                if not moving:
                    forward_start(); moving = True
            else:
                if moving:
                    stop(); moving = False

            frame_count += 1

    except Exception as e:
        print("Crash:", e)
        traceback.print_exc()
    finally:
        stop()
        gpio.cleanup()
        cv2.destroyAllWindows()
        try:
            picam2.close()
        except:
            pass
        print("Clean exit")

if __name__ == "__main__":
    main()
