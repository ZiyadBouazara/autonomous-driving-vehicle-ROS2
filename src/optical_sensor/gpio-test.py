import RPi.GPIO as GPIO
import time
import json

OPTICAL_SENSOR_PIN = 7

def on_pin_edge(event):
    print(event)


GPIO.setmode(GPIO.BOARD)
GPIO.setup(OPTICAL_SENSOR_PIN, GPIO.IN)
# GPIO.add_event_detect(OPTICAL_SENSOR_PIN, GPIO.BOTH, callback=on_pin_edge)



values = []
while True:
    try:
        value = GPIO.input(OPTICAL_SENSOR_PIN)
        print(value)
        values.append(value)
        time.sleep(0.1)
    except KeyboardInterrupt:
        print("Saving data")
        with open("data.json", "w") as f:
            json.dump(values, f)
        GPIO.cleanup()
        break