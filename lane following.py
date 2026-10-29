from machine import Pin, I2C
import neopixel
import time
import math
import veml6040
from motor import Motor

STATES = ("red", "green", "blue", "ready", "follow")
LED = {"red": (40, 0, 0), "green": (0, 40, 0), "blue": (0, 0, 40), "ready": (30, 30, 30), "follow": (0, 0, 0)}

BLACK_THRESHOLD = 1200
TARGET = BLACK_THRESHOLD
MAX_DIST = 0.08
CONFIRM = 3
BASE = 45
FWD = 1
COOLDOWN_MS = 800
DEBOUNCE_MS = 200

KP = 30
KI = 0
KD = 0.6
DT = 0.04
I_MAX = 1

state = "red"
samples = {"red": [], "green": [], "blue": []}
next_flag = False
sample_flag = False
last_press = 0
last_color = None
streak = 0
cooldown_until = 0
integral = 0
prev_err = None
turn = 0


def press(pin):
    global next_flag, sample_flag, last_press
    now = time.ticks_ms()
    if time.ticks_diff(now, last_press) < DEBOUNCE_MS:
        return
    last_press = now
    if pin == btn_next:
        next_flag = True
    else:
        sample_flag = True


def read():
    r, g, b, w = sensor.read_rgbw()
    s = r + g + b or 1
    return (r / s, g / s, b / s), w


def classify_color(v):
    best, d = None, MAX_DIST
    for c in samples:
        for m in samples[c]:
            dist = math.sqrt((v[0] - m[0]) ** 2 + (v[1] - m[1]) ** 2 + (v[2] - m[2]) ** 2)
            if dist < d:
                best, d = c, dist
    return best


def pid(w):
    global integral, prev_err
    err = (w - TARGET) / TARGET
    integral = max(-I_MAX, min(I_MAX, integral + err * DT))
    deriv = 0 if prev_err is None else (err - prev_err) / DT
    prev_err = err
    out = KP * err + KI * integral + KD * deriv
    return max(-BASE, min(BASE, out))


def reset_pid():
    global integral, prev_err, turn
    integral, prev_err, turn = 0, None, 0


def drive(l, r):
    left.start(FWD, l)
    right.start(FWD, r)


def stop():
    left.stop()
    right.stop()


def maneuver(color):
    stop()
    print("detected", color)
    lights[0] = LED[color]
    lights.write()
    time.sleep_ms(500)


i2c = I2C(0, sda=Pin(21), scl=Pin(22), freq=400000)
sensor = veml6040.VEML6040(i2c)
sensor.set_integration_time(veml6040.IT_40MS)

left = Motor(12, 13, 32, 39)
right = Motor(14, 27, 33, 36)
lights = neopixel.NeoPixel(Pin(15), 2)

btn_next = Pin(34, Pin.IN)
btn_sample = Pin(35, Pin.IN)
btn_next.irq(trigger=Pin.IRQ_FALLING, handler=press)
btn_sample.irq(trigger=Pin.IRQ_FALLING, handler=press)

stop()
print("state:", state)

while True:
    if next_flag:
        next_flag = False
        stop()
        state = "ready" if state == "follow" else STATES[STATES.index(state) + 1]
        reset_pid()
        print("state:", state)

    lights[0] = LED[state]

    if state in samples:
        if sample_flag:
            sample_flag = False
            samples[state].append(read()[0])
            print(state, "samples:", len(samples[state]))

    elif state == "follow":
        v, w = read()
        cooling = time.ticks_diff(time.ticks_ms(), cooldown_until) < 0
        color = None if cooling else classify_color(v)

        if color and color == last_color:
            streak += 1
        else:
            streak = 0
        last_color = color

        if streak >= CONFIRM:
            maneuver(color)
            streak = 0
            reset_pid()
            cooldown_until = time.ticks_add(time.ticks_ms(), COOLDOWN_MS)
        else:
            if not color:
                turn = pid(w)
            drive(BASE + turn, BASE - turn)

    lights.write()
    time.sleep_ms(40)