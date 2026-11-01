# color_calibrator.py
# Standalone script to sample and save normalized RGB and white channel values 
# separately for black, white, red, green, and blue into color_calibration.json.

import time
import json
from machine import Pin, I2C
import veml6040

# ================= Configuration =================
STATES = ("black", "white", "red", "green", "blue")
SAMPLE_READS = 5          # Number of quick readings averaged per button press
DEBOUNCE_MS = 200
CALIB_FILE = "color_calibration.json"

# ================= Hardware Setup =================
i2c = I2C(0, sda=Pin(21), scl=Pin(22), freq=100000)
print("I2C devices:", i2c.scan())
sensor = veml6040.VEML6040(i2c)
sensor.set_integration_time(veml6040.IT_40MS)

btn_next = Pin(34, Pin.IN)
btn_sample = Pin(35, Pin.IN)

# ================= State Variables =================
current_state_idx = 0
samples = {s: [] for s in STATES}
next_flag = False
sample_flag = False
last_press = 0

# ================= Button Interrupts =================
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

btn_next.irq(trigger=Pin.IRQ_FALLING, handler=press)
btn_sample.irq(trigger=Pin.IRQ_FALLING, handler=press)

# ================= Helper Functions =================
def mean(xs):
    return sum(xs) / len(xs)

def read_sensor():
    try:
        r, g, b, w = sensor.read_rgbw()
    except OSError:
        return (0.33, 0.33, 0.33), 0
    s = r + g + b or 1
    # Return normalized RGB fractions and raw white channel
    return (r / s, g / s, b / s), w

def save_to_json():
    try:
        with open(CALIB_FILE, "w") as f:
            json.dump(samples, f)
        print(f"--> Saved calibration data to {CALIB_FILE}")
    except OSError as err:
        print("Failed to save file:", err)

def load_from_json():
    try:
        with open(CALIB_FILE, "r") as f:
            data = json.load(f)
            for s in STATES:
                if s in data:
                    samples[s] = data[s]
        print(f"--> Loaded existing calibration from {CALIB_FILE}")
        for s in STATES:
            print(f"   [{s}]: {len(samples[s])} samples saved")
    except (OSError, ValueError):
        print("No existing calibration file found. Starting fresh.")

# Try loading existing data on boot
load_from_json()

print("\n========================================")
print("  STANDALONE COLOR CALIBRATOR ACTIVE")
print("========================================")
print(f"Current Category: [ {STATES[current_state_idx].upper()} ]")
print("- Press 'next' button to switch category.")
print("- Press 'sample' button to record readings.")
print("----------------------------------------\n")

# ================= Main Loop =================
try:
    while True:
        state = STATES[current_state_idx]
        
        if next_flag:
            next_flag = False
            current_state_idx = (current_state_idx + 1) % len(STATES)
            print(f"\n---> Switched category to: [ {STATES[current_state_idx].upper()} ]")
            
        if sample_flag:
            sample_flag = False
            print(f"Sampling {state}...", end="")
            
            rs_r, rs_g, rs_b, rs_w = [], [], [], []
            for _ in range(SAMPLE_READS):
                (r_frac, g_frac, b_frac), w = read_sensor()
                rs_r.append(r_frac)
                rs_g.append(g_frac)
                rs_b.append(b_frac)
                rs_w.append(w)
                time.sleep_ms(40)
                
            avg_r = mean(rs_r)
            avg_g = mean(rs_g)
            avg_b = mean(rs_b)
            avg_w = mean(rs_w)
            
            # Structure stored: [[r_fraction, g_fraction, b_fraction], white_value]
            sample_entry = [[avg_r, avg_g, avg_b], avg_w]
            samples[state].append(sample_entry)
            
            print(f" Done! ({len(samples[state])} total)")
            print(f"   RGB fractions: [{round(avg_r, 3)}, {round(avg_g, 3)}, {round(avg_b, 3)}]")
            print(f"   White channel (w): {int(avg_w)}")
            
            # Save to separate file immediately
            save_to_json()
            
        time.sleep_ms(50)

except KeyboardInterrupt:
    print("\nCalibrator stopped.")