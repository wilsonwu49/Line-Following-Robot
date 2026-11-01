# Robot main.py
# Merges lane_following.py + station.py (ESP-NOW) + Morse Code Buzzer on top of motor.py and veml6040.py.

import network, espnow, json, math
import uasyncio as asyncio
from machine import Pin, I2C, PWM
import neopixel
import time
import veml6040

try:
    from motor import Motor
except ImportError:         # file may be saved as Motor.py
    from Motor import Motor

# ================= config =================
# --- ESP-NOW ---
HUB_MAC = b'\xff\xff\xff\xff\xff\xff'    # broadcast; replace with the hub's MAC later
JOY_TIMEOUT_MS = 500                     # no joystick packet for this long -> stop (manual mode)

# --- states / LEDs ---
STATES = ("black", "white", "red", "green", "blue", "ready", "follow")
COLORS = ("red", "green", "blue")
LED = {"black": (3, 3, 3), "white": (60, 60, 60),
       "red": (40, 0, 0), "green": (0, 40, 0), "blue": (0, 0, 40),
       "ready": (30, 20, 0), "follow": (0, 0, 0)}

# --- sampling / color detection (brightness decides, RGB confirms) ---
SAMPLE_READS = 5          # reads averaged per button press (hold still ~0.2 s)
MAX_DIST = 0.08           # RGB confirmation: max chromaticity distance to the brightness-chosen color
CONFIRM = 3               # consecutive matching reads before a color is confirmed
LOST_READS = 5            # reads without the color before it counts as gone
W_TOL = 0.25              # brightness: how far outside a class's expected w range still counts
                          #   (fraction of the black-to-white span)
W_TIE = 0.05              # classes this close in brightness are "tied"; RGB picks between them
HOLD_DIST = 0.12          # while on a color, stay on it while its RGB is this close
STARTUP_MS = 1000         # after entering ready/follow, ignore colors this long (follow black only)
DEBOUNCE_MS = 200

# --- "is this black / white?" ---
NEUTRAL_DIST = 0.08       # max RGB distance from the black-to-white range to count as black/white;
                          #   farther than this = "other" = drive straight
DARK_FRAC = 0.10          # readings this close to black (share of black->white) have noisy RGB,
                          #   so they always count as black and are never called "other"
WHITE_FRAC = 0.75         # for printing only: w above this share of black->white says "white"

# --- line following ---
EDGE_POS = 0.5            # 0 = sit on black, 1 = sit on white; 0.5 = middle of the edge
BASE = 45
FWD = 1
KP = 30
KI = 0
KD = 0.6
DT = 0.04
I_MAX = 1
PRINT_EVERY = 1           # print the surface every N loops (1 = every loop, 5 = about 5 per second)
MAX_SENSOR_ERRORS = 5     # failed sensor reads in a row before the motors stop and the sensor is reset

# --- manual joystick drive (raw 16-bit values from the hub) ---
JOY_CENTER = 30000       # stick at rest (hub gesture thresholds are 20000 / 40000)
JOY_SPAN = 30000         # distance from center to full deflection
JOY_DEADZONE = 5000
JOY_PRESS = 65000        # x at max = joystick button, ignore it for driving
JOY_X_SIGN = 1           # flip to -1 if steering is reversed
JOY_Y_SIGN = 1           # flip to -1 if forward/back is reversed
MANUAL_MAX = 60          # max motor percent in manual mode

# --- Buzzer & Morse Code Frequencies ---
COLOR_FREQS = {"red": 400, "green": 700, "blue": 1000}
DOT_DURATION = 120
DASH_DURATION = DOT_DURATION * 3
ELEMENT_GAP = DOT_DURATION
LETTER_GAP = DOT_DURATION * 3

# ================= ESP-NOW setup =================
sta = network.WLAN(network.STA_IF)
sta.active(True)

e = espnow.ESPNow()
e.active(True)
e.add_peer(HUB_MAC)

# latest joystick state from the hub
joy = {"x": JOY_CENTER, "y": JOY_CENTER}
last_rx = None            # ticks_ms of the last joystick packet

# ================= state =================
state = "black"
samples = {s: [] for s in STATES[:5]}     # each entry: ((r, g, b) fractions, white channel)
next_flag = False
sample_flag = False
last_press = 0

# color detection
last_color = None
streak = 0
current_color = None     # confirmed color we are on (None = not on a color)
lost = 0
settle_until = 0         # ticks_ms before which colors are ignored (startup window)
class_w = {}             # average white-channel (brightness) reading of each sampled class

# calibration results / line following
black_w = 0
white_w = 0
black_c = (0.33, 0.33, 0.33)   # average RGB fractions of the black samples
white_c = (0.33, 0.33, 0.33)   # average RGB fractions of the white samples
target = 1
half_span = 1
integral = 0
prev_err = None
turn = 0


# ================= buttons =================
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


# ================= sensing =================
def mean(xs):
    return sum(xs) / len(xs)


sensor_errors = 0        # consecutive failed sensor reads
last_reading = ((0.33, 0.33, 0.33), 0)


def recover_sensor():
    """Re-initialise the VEML6040 after the I2C bus stopped answering."""
    global sensor
    try:
        sensor = veml6040.VEML6040(i2c)
        sensor.set_integration_time(veml6040.IT_40MS)
        return True
    except OSError:
        return False


def read():
    """Read the sensor. An I2C timeout (OSError) must not crash the robot: reuse the
    last good reading, and every MAX_SENSOR_ERRORS failures in a row try to
    re-initialise the sensor. The follow loop stops the motors while it is faulted."""
    global sensor_errors, last_reading
    try:
        r, g, b, w = sensor.read_rgbw()
    except OSError as err:
        sensor_errors += 1
        print("sensor read failed (%d): %s" % (sensor_errors, err))
        if sensor_errors % MAX_SENSOR_ERRORS == 0 and recover_sensor():
            print("sensor recovered")
            sensor_errors = 0
        return last_reading
    sensor_errors = 0
    s = r + g + b or 1
    last_reading = ((r / s, g / s, b / s), w)
    return last_reading


async def take_sample():
    """Average a few reads so one noisy reading doesn't spoil the calibration."""
    rs = []
    for _ in range(SAMPLE_READS):
        rs.append(read())
        await asyncio.sleep_ms(40)
    v = tuple(mean([x[0][i] for x in rs]) for i in range(3))
    w = mean([x[1] for x in rs])
    return v, w


def chroma_dist(v, c):
    """Smallest RGB-fraction distance from v to any sample of class c."""
    d = 1e9
    for m, _w in samples[c]:
        dist = math.sqrt((v[0] - m[0]) ** 2 + (v[1] - m[1]) ** 2 + (v[2] - m[2]) ** 2)
        if dist < d:
            d = dist
    return d


def neutral_dist(v):
    """Distance from v to the line between the black and white RGB averages.
    Black, white and every mix of the two (the edge) are close to it; colors are not."""
    ab = [white_c[i] - black_c[i] for i in range(3)]
    ap = [v[i] - black_c[i] for i in range(3)]
    ab2 = sum(x * x for x in ab) or 1e-9
    t = max(0, min(1, sum(ap[i] * ab[i] for i in range(3)) / ab2))
    return math.sqrt(sum((ap[i] - t * ab[i]) ** 2 for i in range(3)))


def classify_color(v, w, hold=None):
    if hold and samples[hold] and chroma_dist(v, hold) < HOLD_DIST:
        return hold

    span = white_w - black_w
    if span <= 0 or not class_w:
        return None

    errs = {}
    for c in class_w:
        cw = class_w[c]
        hi = cw + EDGE_POS * (white_w - cw) if cw < white_w else cw
        err = max(0, cw - w, w - hi) / span
        if err <= W_TOL:
            errs[c] = err
    if not errs:
        return None

    best_err = min(errs.values())
    cls, cls_d = None, 1e9
    for c in errs:
        if errs[c] <= best_err + W_TIE:
            d = chroma_dist(v, c)
            if d < cls_d:
                cls, cls_d = c, d

    if cls in COLORS and cls_d < MAX_DIST:
        return cls
    return None


def settling():
    return time.ticks_diff(time.ticks_ms(), settle_until) < 0


def detect(v, w):
    """Return (color, confirmed). confirmed is True once a color has matched
    on CONFIRM+1 consecutive reads."""
    global last_color, streak
    color = classify_color(v, w, current_color)

    if color and color == last_color:
        streak += 1
    else:
        streak = 0
    last_color = color

    if streak >= CONFIRM:
        streak = 0
        return color, True
    return color, False


def detect_settled(v, w):
    global streak, last_color
    if settling():
        streak, last_color = 0, None
        return None, False
    return detect(v, w)


def report(color, v):
    """Send the color name and an RGB value (hue scaled to 0-255) to the hub."""
    m = max(v) or 1
    msg = {"name": color,
           "r": int(v[0] * 255 / m),
           "g": int(v[1] * 255 / m),
           "b": int(v[2] * 255 / m)}
    try:
        e.send(HUB_MAC, json.dumps(msg), False)
        print("sent:", msg)
    except OSError as err:
        print("send failed:", err)


# ================= calibration results =================
def set_target():
    global target, half_span, prev_err
    target = black_w + EDGE_POS * (white_w - black_w)
    half_span = (white_w - black_w) / 2
    prev_err = None


def compute_targets():
    global black_w, white_w, black_c, white_c
    if not samples["black"] or not samples["white"]:
        return False
    bw = mean([s[1] for s in samples["black"]])
    ww = mean([s[1] for s in samples["white"]])
    if ww <= bw:
        print("white must read brighter than black - resample")
        return False
    black_w, white_w = bw, ww
    black_c = tuple(mean([s[0][i] for s in samples["black"]]) for i in range(3))
    white_c = tuple(mean([s[0][i] for s in samples["white"]]) for i in range(3))

    class_w.clear()
    for c in STATES[:5]:
        if samples[c]:
            class_w[c] = mean([s[1] for s in samples[c]])
    print("brightness (w):", {c: int(x) for c, x in class_w.items()})

    set_target()
    print("black w:", int(black_w), "white w:", int(white_w), "target:", int(target))
    return True


def reset_color_tracking():
    global last_color, streak, current_color, lost
    last_color, streak, current_color, lost = None, 0, None, 0


def track_color(color, confirmed, v):
    """Report a newly confirmed color to the hub once, play Morse code, 
    and remember that we are on it until it has been missing for LOST_READS reads."""
    global current_color, lost
    if confirmed and color != current_color:
        current_color = color
        lost = 0
        report(color, v)
        if color in COLOR_FREQS:
            asyncio.create_task(play_morse_word_async(color, COLOR_FREQS[color]))
    elif current_color:
        if color == current_color:
            lost = 0
        else:
            lost += 1
            if lost >= LOST_READS:
                current_color = None


# ================= motors =================
def pid(w):
    global integral, prev_err
    err = (w - target) / half_span
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


def set_motor(m, speed):
    speed = max(-MANUAL_MAX, min(MANUAL_MAX, int(speed)))
    if speed == 0:
        m.stop()
    elif speed > 0:
        m.start(FWD, speed)
    else:
        m.start(0 if FWD else 1, -speed)


def axis(raw):
    d = raw - JOY_CENTER
    if abs(d) < JOY_DEADZONE:
        return 0
    d = max(-JOY_SPAN, min(JOY_SPAN, d))
    mag = (abs(d) - JOY_DEADZONE) * 100 // (JOY_SPAN - JOY_DEADZONE)
    return mag if d > 0 else -mag


def manual_drive():
    if last_rx is None or time.ticks_diff(time.ticks_ms(), last_rx) > JOY_TIMEOUT_MS:
        stop()
        return
    x = joy["x"]
    if x >= JOY_PRESS:
        x = JOY_CENTER
    fwd = JOY_Y_SIGN * axis(joy["y"])
    steer = JOY_X_SIGN * axis(x)
    set_motor(left, fwd + steer)
    set_motor(right, fwd - steer)


# ================= asynchronous buzzer / morse code =================
async def play_signal_async(duration_ms, frequency):
    buzzer.freq(frequency)
    buzzer.duty(512)  # 50% duty cycle
    await asyncio.sleep_ms(duration_ms)
    buzzer.duty(0)    # Turn off sound
    await asyncio.sleep_ms(ELEMENT_GAP)


async def play_morse_word_async(word, frequency):
    """Asynchronously decodes a word text string and transmits it via Morse code tones."""
    for letter in word.upper():
        if letter == 'R':  # • — •
            await play_signal_async(DOT_DURATION, frequency)
            await play_signal_async(DASH_DURATION, frequency)
            await play_signal_async(DOT_DURATION, frequency)
        elif letter == 'E': # •
            await play_signal_async(DOT_DURATION, frequency)
        elif letter == 'D': # — • •
            await play_signal_async(DASH_DURATION, frequency)
            await play_signal_async(DOT_DURATION, frequency)
            await play_signal_async(DOT_DURATION, frequency)
        elif letter == 'B': # — • • •
            await play_signal_async(DASH_DURATION, frequency)
            await play_signal_async(DOT_DURATION, frequency)
            await play_signal_async(DOT_DURATION, frequency)
            await play_signal_async(DOT_DURATION, frequency)
        elif letter == 'L': # • — • •
            await play_signal_async(DOT_DURATION, frequency)
            await play_signal_async(DASH_DURATION, frequency)
            await play_signal_async(DOT_DURATION, frequency)
            await play_signal_async(DOT_DURATION, frequency)
        elif letter == 'G': # — — •
            await play_signal_async(DASH_DURATION, frequency)
            await play_signal_async(DASH_DURATION, frequency)
            await play_signal_async(DOT_DURATION, frequency)
        
        await asyncio.sleep_ms(LETTER_GAP - ELEMENT_GAP)
    buzzer.duty(0)


# ================= hardware =================
i2c = I2C(0, sda=Pin(21), scl=Pin(22), freq=100000)
print("I2C devices:", i2c.scan())
sensor = veml6040.VEML6040(i2c)
sensor.set_integration_time(veml6040.IT_40MS)

left = Motor(12, 13, 32, 39)
right = Motor(14, 27, 33, 36)
lights = neopixel.NeoPixel(Pin(15), 2)

btn_next = Pin(34, Pin.IN)
btn_sample = Pin(35, Pin.IN)
btn_next.irq(trigger=Pin.IRQ_FALLING, handler=press)
btn_sample.irq(trigger=Pin.IRQ_FALLING, handler=press)

# Passive buzzer on Pin 23
buzzer = PWM(Pin(23, Pin.OUT), freq=1000, duty=0)


# ================= tasks =================
def on_answer(ans):
    print("Hub answer:", ans)


async def receive_task():
    global last_rx
    while True:
        while True:
            mac, raw = e.irecv(0)
            if mac is None:
                break
            try:
                d = json.loads(raw)
                joy["x"], joy["y"] = d["x"], d["y"]
                last_rx = time.ticks_ms()
                if "a" in d:
                    on_answer(d["a"])
            except (ValueError, KeyError):
                print("bad packet:", raw)
        await asyncio.sleep_ms(20)


async def control_task():
    global state, next_flag, sample_flag, turn, settle_until

    stop()
    print("state:", state)
    loops = 0

    while True:
        if next_flag:
            next_flag = False
            stop()
            idx = STATES.index(state)
            new = "ready" if state == "follow" else STATES[idx + 1]

            if new in ("ready", "follow"):
                ok = compute_targets()
                if new == "follow" and not ok:
                    print("Need black and white samples first - back to black")
                    new = "black"

            state = new
            reset_pid()
            reset_color_tracking()
            settle_until = time.ticks_add(time.ticks_ms(), STARTUP_MS)
            print("state:", state)

        lights[0] = LED[current_color] if current_color else LED[state]

        if state in samples:
            if sample_flag:
                sample_flag = False
                samples[state].append(await take_sample())
                v_s, w_s = samples[state][-1]
                print(state, "samples:", len(samples[state]),
                      [round(x, 3) for x in v_s], "w:", int(w_s))

        elif state == "ready":
            v, w = read()
            color, confirmed = detect_settled(v, w)
            track_color(color, confirmed, v)
            manual_drive()

        elif state == "follow":
            v, w = read()
            if sensor_errors >= MAX_SENSOR_ERRORS:
                stop()
                await asyncio.sleep_ms(100)
                continue
            color, confirmed = detect_settled(v, w)
            track_color(color, confirmed, v)

            on_color = color or current_color
            if on_color:
                surface = on_color
            elif (not settling()
                  and w > black_w + DARK_FRAC * (white_w - black_w)
                  and neutral_dist(v) > NEUTRAL_DIST):
                surface = "other"
            else:
                surface = "white" if w > black_w + WHITE_FRAC * (white_w - black_w) else "black"

            if surface in ("black", "white"):
                turn = pid(w)
                drive(BASE + turn, BASE - turn)
                action = "turn %d" % int(turn)
            else:
                reset_pid()
                drive(BASE, BASE)
                action = "straight"

            loops += 1
            if loops % PRINT_EVERY == 0:
                print("line:", surface, "w:", int(w), action)

        lights.write()
        await asyncio.sleep_ms(40)


async def main():
    asyncio.create_task(receive_task())
    await control_task()


try:
    asyncio.run(main())
finally:
    stop()
    buzzer.deinit()
    print("Program stopped.")