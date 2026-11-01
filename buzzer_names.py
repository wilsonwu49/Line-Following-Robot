import time
from machine import Pin, PWM

# Configure the passive buzzer on Pin 23
buzzer_pin = Pin(23, Pin.OUT)
# Initialize PWM with a default frequency of 1000Hz and duty cycle 0 (off)
buzzer = PWM(buzzer_pin, freq=1000, duty=0)

# Morse Code Timing Configuration (in milliseconds)
DOT_DURATION = 120
DASH_DURATION = DOT_DURATION * 3
ELEMENT_GAP = DOT_DURATION
LETTER_GAP = DOT_DURATION * 3
WORD_GAP = DOT_DURATION * 7

def send_signal(duration_ms, frequency):
    """Generates a single Morse code dot or dash tone."""
    buzzer.freq(frequency)
    buzzer.duty(512)  # 50% duty cycle (10-bit resolution on ESP32 MicroPython)
    time.sleep_ms(duration_ms)
    
    buzzer.duty(0)    # Turn off the sound
    time.sleep_ms(ELEMENT_GAP)

def play_morse_word(word, frequency):
    """Decodes a word text string and transmits it via Morse code tones."""
    for letter in word.upper():
        if letter == 'R':   # • — •
            send_signal(DOT_DURATION, frequency)
            send_signal(DASH_DURATION, frequency)
            send_signal(DOT_DURATION, frequency)
        elif letter == 'E': # •
            send_signal(DOT_DURATION, frequency)
        elif letter == 'D': # — • •
            send_signal(DASH_DURATION, frequency)
            send_signal(DOT_DURATION, frequency)
            send_signal(DOT_DURATION, frequency)
        elif letter == 'B': # — • • •
            send_signal(DASH_DURATION, frequency)
            send_signal(DOT_DURATION, frequency)
            send_signal(DOT_DURATION, frequency)
            send_signal(DOT_DURATION, frequency)
        elif letter == 'L': # • — • •
            send_signal(DOT_DURATION, frequency)
            send_signal(DASH_DURATION, frequency)
            send_signal(DOT_DURATION, frequency)
            send_signal(DOT_DURATION, frequency)
        elif letter == 'U': # • • —
            send_signal(DOT_DURATION, frequency)
            send_signal(DOT_DURATION, frequency)
            send_signal(DASH_DURATION, frequency)
        elif letter == 'G': # — — •
            send_signal(DASH_DURATION, frequency)
            send_signal(DASH_DURATION, frequency)
            send_signal(DOT_DURATION, frequency)
        elif letter == 'N': # — •
            send_signal(DASH_DURATION, frequency)
            send_signal(DOT_DURATION, frequency)
            
        # Add space between letters (minus the element gap already applied)
        time.sleep_ms(LETTER_GAP - ELEMENT_GAP)

# Main loop execution
try:
    print("Cytron Robo ESP32 MicroPython Morse Code Color Tester Active")
    while True:
        # 🔴 RED (Low Pitch: 400Hz)
        print("Buzzing 'RED'...")
        play_morse_word("RED", 400)
        time.sleep_ms(WORD_GAP)

        # 🟢 GREEN (Medium Pitch: 700Hz)
        print("Buzzing 'GREEN'...")
        play_morse_word("GREEN", 700)
        time.sleep_ms(WORD_GAP)

        # 🔵 BLUE (High Pitch: 1000Hz)
        print("Buzzing 'BLUE'...")
        play_morse_word("BLUE", 1000)
        
        time.sleep_ms(5000)  # Wait 5 seconds before repeating sequence

except KeyboardInterrupt:
    # Safely deinitialize the PWM channel if stopped in Thonny IDE
    buzzer.deinit()
    print("Program stopped.")
