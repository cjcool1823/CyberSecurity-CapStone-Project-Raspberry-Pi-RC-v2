import os, sys
# suppress libpng “iCCP: known incorrect sRGB profile” warnings
sys.stderr = open(os.devnull, 'w')

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
    for p in MOTOR_PINS:
        gpio.setup(p, gpio.OUT)
    gpio.setup(TRIG, gpio.OUT)
    gpio.setup(ECHO, gpio.IN)

# continuous-drive primitives (no sleeps)
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
    time.sleep(0.00005)
    gpio.output(TRIG, False)

    start = time.time()
    timeout = start + 0.02
    while gpio.input(ECHO) == 0:
        start = time.time()
        if start > timeout:
            return 999
    end = time.time()
    timeout = end + 0.02
    while gpio.input(ECHO) == 1:
        end = time.time()
        if end > timeout:
            return 999

    return round((end - start) * 17150, 2)

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

    try:
        while True:
            frame = picam2.capture_array()
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            disp  = frame.copy()

            # 1) detect legs/shoes
            rects, _ = hog.detectMultiScale(
                frame, winStride=(4,4), padding=(8,8), scale=1.01
            )
            filtered = [(x,y,w,h) for x,y,w,h in rects if w>=30 and h>=60]

            # 2) pick largest centered box
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

            # 3) draw shirt region (upper 40%)
            if best_box:
                x,y,w,h = best_box
                sy = y + int(0.1*h)
                sh = int(0.4*h)
                cv2.rectangle(disp, (x,sy), (x+w,sy+sh), (255,0,0), 2)

            # 4) measure & display distance
            dist = measure_distance()
            cv2.putText(disp, f"D={dist}cm", (10,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

            # 5) show frame
            cv2.imshow("Tracking", cv2.cvtColor(disp, cv2.COLOR_RGB2BGR))
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # 6) AUTO-drive: only if clear path & person seen
            if dist > 30 and person:
                if not moving:
                    forward_start()
                    moving = True
            else:
                if moving:
                    stop()
                    moving = False

    finally:
        stop()
        gpio.cleanup()
        cv2.destroyAllWindows()
        picam2.close()
        print("Program ended")

if __name__ == "__main__":
    main()
