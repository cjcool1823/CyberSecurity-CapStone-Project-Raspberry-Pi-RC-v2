import RPi.GPIO as GPIO
from picamera2 import Picamera2
import time

# Define GPIO pins for the ultrasonic sensor
TRIG_PIN = 6 # Changed from 23 to 5
ECHO_PIN = 5

# Define a distance threshold (in centimeters) to trigger the camera
DISTANCE_THRESHOLD = 50

# Define the filename for the captured image
IMAGE_FILENAME = "captured_image.jpg"

def setup_gpio():
    """Sets up the GPIO pins for the ultrasonic sensor."""
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TRIG_PIN, GPIO.OUT)
    GPIO.setup(ECHO_PIN, GPIO.IN)
    GPIO.output(TRIG_PIN, False)
    time.sleep(2)  # Allow sensor to settle

def measure_distance():
    """Measures the distance using the ultrasonic sensor."""
    # Send a short trigger pulse
    GPIO.output(TRIG_PIN, True)
    time.sleep(0.00001)
    GPIO.output(TRIG_PIN, False)

    pulse_start_time = time.time()
    pulse_end_time = time.time()

    # Measure the duration of the echo pulse
    max_time = 0.04  # Maximum time to wait for echo (adjust as needed)
    start_time = time.time()
    while GPIO.input(ECHO_PIN) == 0:
        pulse_start_time = time.time()
        if (pulse_start_time - start_time) > max_time:
            return 999  # Timeout

    start_time = time.time()
    while GPIO.input(ECHO_PIN) == 1:
        pulse_end_time = time.time()
        if (pulse_end_time - start_time) > max_time:
            return 999  # Timeout

    pulse_duration = pulse_end_time - pulse_start_time
    distance_cm = (pulse_duration * 34300) / 2  # Speed of sound is approx 343 m/s

    return distance_cm

def capture_image():
    """Captures an image using the Raspberry Pi Camera V2."""
    picam2 = Picamera2()
    try:
        picam2.start()
        time.sleep(2)  # Allow camera to warm up
        picam2.capture_file(IMAGE_FILENAME)
        print(f"Image captured: {IMAGE_FILENAME}")
    finally:
        picam2.close()

if __name__ == "__main__":
    try:
        setup_gpio()
        picam2 = Picamera2() # Initialize picamera2 here for potential early errors
        picam2.close() # Close it as we only need it when triggered
        print("Ultrasonic sensor and camera ready.")

        while True:
            distance = measure_distance()
            print(f"Distance: {distance:.2f} cm")

            if distance <= DISTANCE_THRESHOLD:
                print("Object detected within threshold! Capturing image...")
                capture_image()
                time.sleep(5)  # Wait a bit before checking again

            time.sleep(0.1)  # Small delay between measurements

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        GPIO.cleanup()
