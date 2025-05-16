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
    for p in MOTOR_PINS: gpio.output(p, False)

# manual‐mode pulses
def forward(duration=0.5):
    forward_start(); time.sleep(duration); stop()

def backward(duration=0.5):
    backward_start(); time.sleep(duration); stop()

def left(duration=0.5):
    left_start(); time.sleep(duration); stop()

def right(duration=0.5):
    right_start(); time.sleep(duration); stop()

def measure_distance():
    # faster trigger cycle
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

# set up camera + HOG
picam2 = Picamera2()
picam2.preview_configuration.main.size   = (640, 480)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate = 30    # ↑ from 16 → 30 fps
picam2.configure("preview")
picam2.start()
time.sleep(0.05)

hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

init()
moving = False
manual_mode = False

try:
    while True:
        frame = picam2.capture_array()
        frame = cv2.rotate(frame, cv2.ROTATE_180)

        # detect legs/shoes
        rects, _ = hog.detectMultiScale(
            frame, winStride=(4,4), padding=(8,8), scale=1.01
        )
        filtered = [(x,y,w,h) for x,y,w,h in rects if w>=30 and h>=60]

        # pick largest center‐box
        person = False; best = 0
        for x,y,w,h in filtered:
            cx = x + w//2
            area = w*h
            if 220<cx<420 and area>best:
                person = True; best=area

        dist = measure_distance()

        # draw & display
        disp = frame.copy()
        for x,y,w,h in filtered:
            cv2.rectangle(disp,(x,y),(x+w,y+h),(0,255,0),2)
        cv2.putText(disp,f"D={dist}cm",(10,20),
                    cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,0,255),2)
        cv2.imshow("Tracking", disp)

        key = cv2.waitKey(1) & 0xFF
        if key==ord('m'):
            manual_mode = not manual_mode
            stop(); moving=False
            print("Manual mode:", manual_mode)

        if manual_mode:
            if key==82: forward()
            if key==84: backward()
            if key==81: left()
            if key==83: right()
            if key==ord('s'): stop()
            if key==ord('q'): break
        else:
            # AUTO MODE: stop if obstacle, else full‐power forward
            if dist<=30 or not person:
                stop(); moving=False
            else:
                forward_start(); moving=True

            if key==ord('q'): break

finally:
    stop()
    gpio.cleanup()
    cv2.destroyAllWindows()
    picam2.close()
    print("Ended")
