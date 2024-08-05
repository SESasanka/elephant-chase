### Setting Up the Raspberry Pi

# Install the necessary libraries if you haven’t already:

# bashkkk
# sudo apt-get update
# sudo apt-get install python3-pip
# pip3 install numpy sounddevice RPi.GPIO


### Python Code

# Here’s the updated Python code to include the HC-SR04 sensor:

import numpy as np
import sounddevice as sd
import RPi.GPIO as GPIO
import time
import threading

# Sound Parameters
duration = 5.0  # seconds
frequency = 15.0  # Hz, typical for elephant infrasound
sample_rate = 44100  # samples per second

# GPIO Setup
led_pins = [17, 18, 27, 22, 23, 24, 25, 4]  # GPIO pins for the LEDs
trig_pin = 5  # GPIO pin for HC-SR04 Trigger
echo_pin = 6  # GPIO pin for HC-SR04 Echo

GPIO.setmode(GPIO.BCM)
for pin in led_pins:
    GPIO.setup(pin, GPIO.OUT)
GPIO.setup(trig_pin, GPIO.OUT)
GPIO.setup(echo_pin, GPIO.IN)

# Generate the time points
t = np.linspace(0, duration, int(sample_rate * duration), endpoint=False)

# Generate the sine wave
waveform = 0.5 * np.sin(2 * np.pi * frequency * t)


def play_sound():
    # Play the sound
    sd.play(waveform, sample_rate)
    sd.wait()


def led_chase():
    # LED chase effect
    for _ in range(10):  # Run the chase sequence 10 times
        for pin in led_pins:
            GPIO.output(pin, GPIO.HIGH)
            time.sleep(0.1)
            GPIO.output(pin, GPIO.LOW)


def measure_distance():
    # Measure distance using HC-SR04
    GPIO.output(trig_pin, GPIO.LOW)
    time.sleep(2)

    GPIO.output(trig_pin, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trig_pin, GPIO.LOW)

    while GPIO.input(echo_pin) == 0:
        start_time = time.time()

    while GPIO.input(echo_pin) == 1:
        end_time = time.time()

    duration = end_time - start_time
    distance = (duration * 34300) / 2  # Speed of sound: 34300 cm/s
    return distance


try:
    while True:
        distance = measure_distance()
        print(f"Distance: {distance:.2f} cm")

        if distance < 100:  # If the object is within 100 cm
            sound_thread = threading.Thread(target=play_sound)
            led_thread = threading.Thread(target=led_chase)

            sound_thread.start()
            led_thread.start()

            sound_thread.join()
            led_thread.join()

        time.sleep(1)  # Wait a second before measuring again
finally:
    GPIO.cleanup()

