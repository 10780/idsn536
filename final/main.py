# Device Firmware Code
# This firmware is designed for an ESP32 microcontroller.
# It handles input from microphone, sensors, LEDs, haptic motor, and network communication.

from machine import Pin, I2C, PWM, ADC, Timer
from mpu6050 import MPU6050  # Assuming you have the MPU6050 library installed
import network
import time
import urequests
import esp
import uio

# Disable debug messages
esp.osdebug(None)

# Network Credentials
WIFI_SSID = 'your_ssid'
WIFI_PASSWORD = 'your_password'

# Server URLs
UPLOAD_URL = 'http://192.168.1.100:8000/upload' # Replace with your server's URL
DOWNLOAD_URL = 'http://192.168.1.100:8000/download' # Replace with your server's URL

# Connect to Wi-Fi
station = network.WLAN(network.STA_IF)
station.active(True)
station.connect(WIFI_SSID, WIFI_PASSWORD)

while not station.isconnected():
    time.sleep(1)

print('Connected to WiFi, IP Address:', station.ifconfig()[0])

# Define Pins
MIC_PIN = 34           # Microphone (Analog Input)
SPEAKER_PIN = 26       # Speaker (PWM Output)
HAPTIC_MOTOR_PIN = 27  # Vibration Motor
LED_PIN = 25           # LED Output
TOUCH_PIN = 4          # Capacitive Touch Sensor
PRESSURE_PIN = 32      # Pressure Sensor (Analog Input)
I2C_SCL = 22           # I2C Clock
I2C_SDA = 21           # I2C Data

# Initialize Components
mic = ADC(Pin(MIC_PIN))
speaker = PWM(Pin(SPEAKER_PIN))
speaker.freq(1000)
haptic_motor = Pin(HAPTIC_MOTOR_PIN, Pin.OUT)
led = Pin(LED_PIN, Pin.OUT)
touch = Pin(TOUCH_PIN, Pin.IN)
pressure = ADC(Pin(PRESSURE_PIN))

# Initialize MPU6050
i2c = I2C(0, scl=Pin(I2C_SCL), sda=Pin(I2C_SDA))
mpu = MPU6050(i2c)

# Device State
state = 'IDLE'
recording_buffer = bytearray()
recording = False


def start_recording():
    global state, recording_buffer, recording
    print("Recording...")
    led.value(1)
    state = 'RECORDING'
    recording_buffer = bytearray()
    recording = True


def stop_recording():
    global state, recording
    led.value(0)
    state = 'IDLE'
    recording = False
    print("Recording complete")


def record_advice():
    global recording_buffer, recording

    if recording:
        audio_sample = mic.read() >> 2
        recording_buffer.append(audio_sample)


def playback_advice():
    global state
    print("Playing back advice")
    led.value(1)
    state = 'PLAYBACK'

    for sample in recording_buffer:
        speaker.duty(sample)
        time.sleep(0.01)

    speaker.duty(0)
    led.value(0)
    state = 'IDLE'
    print("Playback complete")


def send_advice():
    global state, recording_buffer
    print("Sending advice")
    state = 'SENDING'
    haptic_motor.value(1)
    time.sleep(0.5)
    haptic_motor.value(0)
    try:
        response = urequests.post(UPLOAD_URL, data=recording_buffer)
        print('Upload Response:', response.text)
    except Exception as e:
        print('Failed to send advice:', e)
    state = 'IDLE'


def receive_advice():
    global state, recording_buffer
    print("Receiving advice")
    state = 'RECEIVING'
    try:
        response = urequests.get(DOWNLOAD_URL)
        if response.status_code == 200:
            recording_buffer = response.content
            print('Advice received successfully')
        else:
            print('Failed to receive advice')
    except Exception as e:
        print('Error during receiving advice:', e)
    state = 'IDLE'


def handle_gesture():
    if pressure.read() > 1000:  # Squeeze gesture detected
        print("Thank you gesture detected")
        haptic_motor.value(1)
        time.sleep(0.2)
        haptic_motor.value(0)


def detect_motion():
    accel_data = mpu.get_accel_data()
    if abs(accel_data['x']) > 1 or abs(accel_data['y']) > 1 or abs(accel_data['z']) > 1:
        print("Motion detected")
        return True
    return False


def loop():
    global state, recording
    while True:
        if state == 'IDLE':
            if touch.value() == 1 and not recording:
                start_recording()
            elif detect_motion():
                playback_advice()
            elif pressure.read() > 1000:  # Replace with appropriate threshold
                handle_gesture()
        elif state == 'RECORDING':
            record_advice()
            if touch.value() == 0 and recording:
                stop_recording()

        if state == 'IDLE':
            receive_advice()  # Periodically check for new advice

        time.sleep(0.01)


if __name__ == "__main__":
    loop()
