import network, espnow, json
import uasyncio as asyncio
from machine import Pin, I2C
import veml6040

# ---------- config ----------
STATION_MAC = b'\xff\xff\xff\xff\xff\xff'   # broadcast; replace with STATION's MAC later
REPORT_MS = 100                         # sensor read rate
STABLE_READS = 3                        # same name this many reads in a row before sending
DARK_WHITE = 300                        # white channel below this = "dark" (tune!)
DEBUG = True                            # print raw sensor values while tuning

# ---------- ESP-NOW setup ----------
sta = network.WLAN(network.STA_IF)
sta.active(True)

e = espnow.ESPNow()
e.active(True)
e.add_peer(STATION_MAC)

# ---------- sensor setup ----------
i2c = I2C(0, sda=Pin(21), scl=Pin(22), freq=400000)
print("I2C devices:", i2c.scan())
sensor = veml6040.VEML6040(i2c)
sensor.trigger_measurement()

# latest joystick state from the STATION
joy = {"x": 0, "y": 0, "a": None}


# ---------- color sensing ----------
def read_color():
    """Return (r, g, b, w) raw, as 0-65535 values from the sensor."""
    r, g, b, w = sensor.read_rgbw()
    return r, g, b, w


def to_rgb255(r, g, b):
    """Scale so the strongest channel is 255 (keeps hue, drops brightness)."""
    m = max(r, g, b, 1)
    return (r * 255 // m, g * 255 // m, b * 255 // m)


def classify(r, g, b, w):
    """Name the color from channel ratios. Tune the numbers with DEBUG on."""
    if w < DARK_WHITE:
        return "dark"
    total = max(r + g + b, 1)
    fr, fg, fb = r / total, g / total, b / total

    if fr > 0.45 and fr > fg * 1.5 and fr > fb * 1.5:
        return "red"
    if fg > 0.42 and fg > fr * 1.2 and fg > fb * 1.2:
        return "green"
    if fb > 0.42 and fb > fr * 1.2 and fb > fg * 1.2:
        return "blue"
    if fr > 0.35 and fg > 0.35 and fb < 0.2:
        return "yellow"
    if max(fr, fg, fb) < 0.4:
        return "white"      # roughly balanced channels
    return "unknown"


# ---------- tasks ----------
async def receive_task():
    while True:
        while True:
            mac, raw = e.irecv(0)
            if mac is None:
                break
            try:
                d = json.loads(raw)
                joy["x"], joy["y"] = d["x"], d["y"]
                if "a" in d:
                    joy["a"] = d["a"]
                    print("STATION answer:", d["a"])
            except (ValueError, KeyError):
                print("bad packet:", raw)
        # TODO: drive motors from joy["x"], joy["y"] here
        await asyncio.sleep_ms(20)


async def color_task():
    sent_name = None
    candidate = None
    count = 0
    while True:
        r, g, b, w = read_color()
        name = classify(r, g, b, w)
        if DEBUG:
            print(r, g, b, w, "->", name)

        # debounce: only act once the same name repeats a few reads in a row
        if name == candidate:
            count += 1
        else:
            candidate = name
            count = 1

        if count >= STABLE_READS and name != sent_name:
            r8, g8, b8 = to_rgb255(r, g, b)
            msg = {"name": name, "r": r8, "g": g8, "b": b8}
            try:
                e.send(STATION_MAC, json.dumps(msg))
                sent_name = name
            except OSError as err:
                print("send failed:", err)

        await asyncio.sleep_ms(REPORT_MS)


async def main():
    asyncio.create_task(receive_task())
    asyncio.create_task(color_task())
    while True:
        await asyncio.sleep(1)

asyncio.run(main())