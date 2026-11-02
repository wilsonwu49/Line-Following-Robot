import network
import espnow
from machine import Pin, I2C
import neopixel
import time
import math
import json
import veml6040
import buzzer
from Motor import Motor

STATION_MAC = b"\x20\x50\x0d\x2b\x1a\x98"
DATA_FILE = "color_data.json"
FRESH = False
COLORS = ("red", "green", "blue")
STATES = ("black", "white", "red", "green", "blue", "ready", "follow")
SOUND = {"red": "mad", "green": "happy", "blue": "sad"}
LED = {"black": (3, 3, 3), "white": (40, 40, 40), "red": (40, 0, 0), "green": (0, 40, 0),
       "blue": (0, 0, 40), "ready": (30, 20, 0), "follow": (0, 0, 0)}

READS = 5
NEUTRAL_CUT = 0.025
COLOR_MAX = 0.05
PATCH_CUT = 0.012
PATCH_MS = 250
EDGE_POS = 0.15
CONFIRM = 1
COOLDOWN_MS = 4000
DEBOUNCE_MS = 200

BASE = 25
L_FWD = 0
R_FWD = 1
KP = 45
KI = 0
KD = 7
DT = 0.04
I_MAX = 1

next_flag = False
sample_flag = False
last_press = 0
last_color = None
streak = 0
cooldown_until = 0
patch_until = 0
integral = 0
prev_err = None
turn = 0
lit = None


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


def load():
    try:
        with open(DATA_FILE) as f:
            return json.load(f)
    except (OSError, ValueError):
        return []


def save():
    try:
        with open(DATA_FILE, "w") as f:
            json.dump(data, f)
    except OSError as e:
        print("save failed:", e)


def dist(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def chroma(r, g, b):
    s = r + g + b or 1
    return (r / s, g / s, b / s)


def calibrate():
    global B_C, W_C, BLACK_W, WHITE_W, COLOR_C
    rows = {l: [d for d in data if d[4] == l] for l in ("black", "white") + COLORS}
    for l in rows:
        if not rows[l]:
            print("missing samples:", l)
            return False

    def avg_c(l):
        cs = [chroma(d[0], d[1], d[2]) for d in rows[l]]
        return [sum(c[i] for c in cs) / len(cs) for i in range(3)]

    def avg_w(l):
        return sum(d[3] for d in rows[l]) / len(rows[l])

    B_C, W_C = avg_c("black"), avg_c("white")
    BLACK_W, WHITE_W = avg_w("black"), avg_w("white")
    COLOR_C = {c: avg_c(c) for c in COLORS}
    if WHITE_W <= BLACK_W:
        print("white must be brighter than black")
        return False
    print("black w=%d  white w=%d" % (BLACK_W, WHITE_W))
    return True


def neutral_dist(v):
    ab = [W_C[i] - B_C[i] for i in range(3)]
    ap = [v[i] - B_C[i] for i in range(3)]
    t = max(0, min(1, sum(ap[i] * ab[i] for i in range(3)) / sum(x * x for x in ab)))
    return math.sqrt(sum((ap[i] - t * ab[i]) ** 2 for i in range(3)))


def classify(r, g, b, w):
    v = chroma(r, g, b)
    wn = (w - BLACK_W) / (WHITE_W - BLACK_W)
    nd = neutral_dist(v)
    if nd > NEUTRAL_CUT:
        best = min(COLOR_C, key=lambda c: dist(v, COLOR_C[c]))
        return (best if dist(v, COLOR_C[best]) < COLOR_MAX else "unknown"), wn, nd
    return "neutral", wn, nd


def pid(wn):
    global integral, prev_err
    err = (wn - EDGE_POS) / (EDGE_POS if wn < EDGE_POS else 1 - EDGE_POS)
    err = max(-1, min(1, err))
    integral = max(-I_MAX, min(I_MAX, integral + err * DT))
    deriv = 0 if prev_err is None else (err - prev_err) / DT
    prev_err = err
    out = KP * err + KI * integral + KD * deriv
    return max(-BASE, min(BASE, out))


def reset_pid():
    global integral, prev_err, turn
    integral, prev_err, turn = 0, None, 0


def drive(l, r):
    left.start(L_FWD, l)
    right.start(R_FWD, r)


def stop():
    left.stop()
    right.stop()


def report(color, r, g, b):
    m = max(r, g, b, 1)
    msg = {"name": color, "r": r * 255 // m, "g": g * 255 // m, "b": b * 255 // m}
    lights[1] = (0, 0, 0)
    try:
        e.send(STATION_MAC, json.dumps(msg), False)
        print("sent:", msg)
    except OSError as err:
        print("send failed:", err)


def poll():
    while True:
        mac, raw = e.irecv(0)
        if mac is None:
            return
        try:
            d = json.loads(raw)
        except ValueError:
            continue
        if "a" in d:
            print("STATION answer:", d["a"])
            lights[1] = (0, 40, 0)


def maneuver(color, r, g, b):
    global lit
    print("detected", color)
    report(color, r, g, b)
    buzzer.play(SOUND[color])
    lit = color


sta = network.WLAN(network.STA_IF)
sta.active(True)
e = espnow.ESPNow()
e.active(True)
e.add_peer(STATION_MAC)

i2c = I2C(0, sda=Pin(21), scl=Pin(22), freq=100000)
sensor = veml6040.VEML6040(i2c)
sensor.set_integration_time(veml6040.IT_40MS)

left = Motor(12, 13, 32, 39)
right = Motor(14, 27, 33, 36)
lights = neopixel.NeoPixel(Pin(15), 2)

btn_next = Pin(34, Pin.IN)
btn_sample = Pin(35, Pin.IN)
btn_next.irq(trigger=Pin.IRQ_FALLING, handler=press)
btn_sample.irq(trigger=Pin.IRQ_FALLING, handler=press)

data = [] if FRESH else load()
calibrate()
state = "ready"
stop()
print("state:", state, " loaded:", len(data), "samples")

try:
    while True:
        if next_flag:
            next_flag = False
            stop()
            buzzer.stop()
            reset_pid()
            streak, last_color, patch_until, lit = 0, None, 0, None
            if state == "follow":
                state = "ready"
            elif state == "ready":
                if calibrate():
                    state = "follow"
                else:
                    print("need samples first, press D35 to train")
            else:
                state = STATES[STATES.index(state) + 1]
                if state == "ready":
                    calibrate()
            print("state:", state)

        poll()
        lights[0] = LED[lit] if state == "follow" and lit else LED[state]

        if state in ("black", "white") + COLORS:
            if sample_flag:
                sample_flag = False
                for _ in range(READS):
                    time.sleep_ms(45)
                    r, g, b, w = sensor.read_rgbw()
                    data.append([r, g, b, w, state])
                save()
                print(state, "samples:", len([d for d in data if d[4] == state]))

        elif state == "ready":
            if sample_flag:
                sample_flag = False
                state = "black"
                print("retrain, state:", state)

        elif state == "follow":
            sample_flag = False
            r, g, b, w = sensor.read_rgbw()
            label, wn, nd = classify(r, g, b, w)
            now = time.ticks_ms()
            cooling = time.ticks_diff(now, cooldown_until) < 0
            color = label if label in COLORS and not cooling else None

            if color and color == last_color:
                streak += 1
            else:
                streak = 0
            last_color = color

            if streak >= CONFIRM:
                maneuver(color, r, g, b)
                streak = 0
                reset_pid()
                cooldown_until = time.ticks_add(time.ticks_ms(), COOLDOWN_MS)
                patch_until = time.ticks_add(time.ticks_ms(), PATCH_MS)
            else:
                if nd > PATCH_CUT:
                    patch_until = time.ticks_add(now, PATCH_MS)
                if time.ticks_diff(now, patch_until) < 0:
                    reset_pid()
                    drive(BASE, BASE)
                else:
                    turn = pid(wn)
                    drive(BASE + turn, BASE - turn)

            print(label, "%.2f" % wn, "%.3f" % nd, int(turn))

        lights.write()
        time.sleep_ms(40)
finally:
    stop()
    buzzer.stop()