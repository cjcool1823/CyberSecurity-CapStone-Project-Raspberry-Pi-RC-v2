import RPi.GPIO as gpio
import time
import cv2
from picamera2 import Picamera2

# ... (rest of your motor and ultrasonic code remains the same) ...

# Camera and people detection setup using picamera2
picam2 = Picamera2()
picam2.preview_configuration.main.size = (640, 480)
picam2.preview_configuration.main.format = "BGR888"
picam2.preview_configuration.controls = {"FrameRate": 16, "WhiteBalance": "incandescent"}

picam2.configure("preview")
picam2.start()
time.sleep(0.1)

print("Showing camera preview for 3 seconds with potential white balance adjustment.")
for _ in range(30):
    frame = picam2.capture_array()
    cv2.imshow("Camera Preview", frame)
    cv2.waitKey(100)
cv2.destroyWindow("Camera Preview")

hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

print("Starting people and obstacle tracking. Press 'q' to quit.")

init()
moving = False

try:
    while True:
        # Adjust white balance in the loop (optional, might affect performance)
        # picam2.set_controls({"WhiteBalance": "daylight"})

        image = picam2.capture_array()

        (rects, weights) = hog.detectMultiScale(
            image,
            winStride=(8, 8),
            padding=(16, 16),
            scale=1.05
        )

        print(f"Detections: {len(rects)}")

        person_detected = False
        for (x, y, w, h) in rects:
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            center_x = x + w // 2
            if 220 < center_x < 420:
                person_detected = True

        distance = measure_distance()
        print(f"Distance: {distance} cm")
        cv2.putText(image, f"Distance: {distance}cm", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)

        if person_detected:
            if moving:
                print("Person detected! Stopping.")
                stop()
                moving = False
        elif distance <= 30:
            if moving:
                print("Obstacle detected! Stopping for safety.")
                stop()
                moving = False
        else:
            if not moving:
                print("Path clear, moving forward")
                forward()
                moving = True

        cv2.imshow("People & Obstacle Tracking", image)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

finally:
    stop()
    gpio.cleanup()
    cv2.destroyAllWindows()
    picam2.close()
    print("Program ended")                                                                                             