# color_tester.py
# Standalone script to test and evaluate the color_calibration.json training data
# by continuously sampling the sensor and printing the matched category.

import time
import json
import math
from machine import Pin, I2C
import veml6040

# ================= Configuration =================
CALIB_FILE = "color_calibration.json"
STATES = ("black", "white", "red", "green", "blue")

# ================= Load Calibration Data =================
samples = {s: [] for s in STATES}
try:
    with open(CALIB_FILE, "r") as f:
        data = json.load(f)
        for s in STATES:
            if s in data:
                samples[s] = data[s]
    print(f"--> Loaded {CALIB_FILE} successfully!")
    for s in STATES:
        print(f"   [{s}]: {len(samples[s])} training samples found")
except (OSError, ValueError):
    print(f"Error: Could not load {CALIB_FILE}. Please run the calibrator script first!")

# ================= Hardware Setup =================
i2c = I2C(0, sda=Pin(21), scl=Pin(22), freq=100000)
print("I2C devices:", i2c.scan())
sensor = veml6040.VEML6040(i2c)
sensor.set_integration_time(veml6040.IT_40MS)

# ================= Helper Functions =================
def read_sensor():
    try:
        r, g, b, w = sensor.read_rgbw()
    except OSError:
        return (0.33, 0.33, 0.33), 0
    s = r + g + b or 1
    return (r / s, g / s, b / s), w

def classify(v, w):
    """Finds the closest matching training category based on Euclidean distance in RGB-fraction space."""
    best_cat = "unknown"
    min_dist = 1e9
    
    for cat, points in samples.items():
        if not points:
            continue
        for pt in points:
            m = pt[0]  # pt[0] is [r_frac, g_frac, b_frac]
            # Euclidean distance across chromaticity components
            dist = math.sqrt((v[0] - m[0]) ** 2 + (v[1] - m[1]) ** 2 + (v[2] - m[2]) ** 2)
            if dist < min_dist:
                min_dist = dist
                best_cat = cat
    return best_cat, min_dist

print("\n========================================")
print("    COLOR CALIBRATION TESTER ACTIVE")
print("========================================\n")

# ================= Main Test Loop =================
try:
    while True:
        v, w = read_sensor()
        match, dist = classify(v, w)
        
        # Print live classification and metrics
        print(f"Matched: [{match.upper():<5}] | Dist: {dist:.3f} | RGB: [{round(v[0],3)}, {round(v[1],3)}, {round(v[2],3)}] | W: {int(w)}")
        
        time.sleep_ms(200) # Print ~5 times per second

except KeyboardInterrupt:
    print("\nTester stopped.")