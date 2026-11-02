from machine import Pin, PWM, Timer
import time

BUZZER_PIN = 23
TICK_MS = 5
TIMER_ID = 0

buzzer = PWM(Pin(BUZZER_PIN), freq=1000, duty_u16=0)


def tone(freq, ms, gap=30):
    return [(freq, ms), (0, gap)]


def growl(ms, low=170, high=220):
    steps = []
    for _ in range(ms // 30):
        steps += [(low, 15), (high, 15)]
    return steps


def happy():
    return tone(523, 90, 20) + tone(659, 90, 20) + tone(784, 90, 20) + tone(1047, 250)


def annoyed():
    return tone(440, 160, 120) + tone(330, 280)


def mad():
    steps = []
    for _ in range(5):
        steps += growl(300) + [(0, 80)]
    return steps


def sad():
    return [(f, 30) for f in range(400, 220, -15)] + [(0, 30)]


SOUNDS = {"happy": happy(), "annoyed": annoyed(), "mad": mad(), "sad": sad()}

_seq = None
_i = 0
_left = 0


def _apply(freq):
    if freq > 0:
        buzzer.freq(freq)
        buzzer.duty_u16(32768)
    else:
        buzzer.duty_u16(0)


def _tick(t):
    global _seq, _i, _left
    if _seq is None:
        return
    _left -= TICK_MS
    if _left > 0:
        return
    _i += 1
    if _i >= len(_seq):
        _seq = None
        buzzer.duty_u16(0)
        return
    f, ms = _seq[_i]
    _apply(f)
    _left = ms


def play(name):
    global _seq, _i, _left
    _seq = None
    seq = SOUNDS[name]
    _i = 0
    _left = seq[0][1]
    _apply(seq[0][0])
    _seq = seq


def stop():
    global _seq
    _seq = None
    buzzer.duty_u16(0)


def playing():
    return _seq is not None


_timer = Timer(TIMER_ID)
_timer.init(period=TICK_MS, mode=Timer.PERIODIC, callback=_tick)


if __name__ == "__main__":
    for name in ("happy", "sad", "mad", "annoyed"):
        print(name)
        play(name)
        while playing():
            time.sleep_ms(50)
        time.sleep_ms(500)