import RPi.GPIO as gpio
import time
import cv2
from picamera2 import Picamera2
import tkinter as tk
from PIL import Image, ImageTk
import threading

# Motor pins
MOTOR_PINS = [17, 22, 23, 24]
TRIG = 5
ECHO = 6

# Modes
MODE_MANUAL = 0
MODE_FOLLOW = 1
MODE_AUTO = 2

current_mode = MODE_MANUAL
running = True

def init():
    gpio.setmode(gpio.BCM)
    for pin in MOTOR_PINS:
        gpio.setup(pin, gpio.OUT)
    gpio.setup(TRIG, gpio.OUT)
    gpio.setup(ECHO, gpio.IN)

def forward(sec=0.5):
    gpio.output(17, False)
    gpio.output(22, True)
    gpio.output(23, True)
    gpio.output(24, False)
    time.sleep(sec)
    stop()

def left(sec=0.3):
    gpio.output(17, True)
    gpio.output(22, False)
    gpio.output(23, True)
    gpio.output(24, False)
    time.sleep(sec)
    stop()

def right(sec=0.3):
    gpio.output(17, False)
    gpio.output(22, True)
    gpio.output(23, False)
    gpio.output(24, True)
    time.sleep(sec)
    stop()

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
            return 999

    pulse_end = time.time()
    timeout = pulse_end + 0.04
    while gpio.input(ECHO) == 1:
        pulse_end = time.time()
        if pulse_end > timeout:
            return 999

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance, 2)
    if distance <= 2 or distance >= 400:
        return 999
    return distance

# Camera and people detection setup using picamera2
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "BGR888"
picam2.preview_configuration.controls.FrameRate = 16
picam2.configure("preview")
picam2.start()
time.sleep(0.1)

hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

init()

# --- Tkinter GUI setup ---
root = tk.Tk()
root.title("Robot Control & Camera View")

# Camera frame
camera_label = tk.Label(root)
camera_label.pack()

# Info labels
distance_label = tk.Label(root, text="Distance: -- cm", font=("Arial", 14))
distance_label.pack()
detections_label = tk.Label(root, text="Detections: --", font=("Arial", 14))
detections_label.pack()
status_label = tk.Label(root, text="Status: --", font=("Arial", 14), fg="blue")
status_label.pack()

def set_manual():
    global current_mode
    current_mode = MODE_MANUAL
    status_label.config(text="Status: Manual Mode")

def set_follow():
    global current_mode
    current_mode = MODE_FOLLOW
    status_label.config(text="Status: Follow Person Mode")

def set_auto():
    global current_mode
    current_mode = MODE_AUTO
    status_label.config(text="Status: Automation Mode")

btn_frame = tk.Frame(root)
btn_frame.pack()
tk.Button(btn_frame, text="Manual", command=set_manual, width=12).pack(side=tk.LEFT)
tk.Button(btn_frame, text="Follow Person", command=set_follow, width=12).pack(side=tk.LEFT)
tk.Button(btn_frame, text="Automation", command=set_auto, width=12).pack(side=tk.LEFT)

def robot_logic():
    global running
    while running:
        image = picam2.capture_array()
        (rects, weights) = hog.detectMultiScale(
            image,
            winStride=(8, 8),
            padding=(16, 16),
            scale=1.05
        )

        person_detected = False
        person_center_x = None
        largest_area = 0
        for (x, y, w, h) in rects:
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            center_x = x + w // 2
            area = w * h
            if area > largest_area:
                largest_area = area
                person_detected = True
                person_center_x = center_x

        distance = measure_distance()
        cv2.putText(image, f"Distance: {distance}cm", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)

        # Update GUI
        img_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        img_pil = Image.fromarray(img_rgb)
        img_tk = ImageTk.PhotoImage(img_pil)
        camera_label.imgtk = img_tk
        camera_label.configure(image=img_tk)
        distance_label.config(text=f"Distance: {distance} cm")
        detections_label.config(text=f"Detections: {len(rects)}")

        # Robot logic based on mode
        if current_mode == MODE_MANUAL:
            status_label.config(text="Status: Manual Mode")
            stop()
        elif current_mode == MODE_FOLLOW:
            if person_detected:
                if person_center_x < 220:
                    status_label.config(text="Status: Turning Left")
                    left()
                elif person_center_x > 420:
                    status_label.config(text="Status: Turning Right")
                    right()
                else:
                    status_label.config(text="Status: Following Forward")
                    forward()
            else:
                status_label.config(text="Status: No Person Detected")
                stop()
        elif current_mode == MODE_AUTO:
            if distance <= 30:
                status_label.config(text="Status: Obstacle Detected! Stopping.")
                stop()
            elif person_detected:
                if person_center_x < 220:
                    status_label.config(text="Status: Turning Left (Auto)")
                    left()
                elif person_center_x > 420:
                    status_label.config(text="Status: Turning Right (Auto)")
                    right()
                else:
                    status_label.config(text="Status: Following Forward (Auto)")
                    forward()
            else:
                status_label.config(text="Status: Path Clear, Moving Forward (Auto)")
                forward()
        else:
            status_label.config(text="Status: Unknown Mode")
            stop()

        time.sleep(0.1)  # Reduce CPU usage

def on_closing():
    global running
    running = False
    stop()
    gpio.cleanup()
    picam2.close()
    root.destroy()

# Run robot logic in a separate thread
robot_thread = threading.Thread(target=robot_logic, daemon=True)
robot_thread.start()

root.protocol("WM_DELETE_WINDOW", on_closing)
root.mainloop()