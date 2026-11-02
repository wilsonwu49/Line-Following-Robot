from machine import Pin, PWM
import uasyncio as asyncio

BUZZER_PIN = 23

buzzer = PWM(Pin(BUZZER_PIN), freq=1000, duty_u16=0)


async def tone(freq, ms, gap=30):
    """Play one note, then a short silence."""
    if freq > 0:
        buzzer.freq(freq)
        buzzer.duty_u16(32768)      # 50% duty = loudest clean tone
    await asyncio.sleep_ms(ms)
    buzzer.duty_u16(0)
    await asyncio.sleep_ms(gap)


async def growl(ms, low=170, high=220):
    """Rough, buzzy sound made by flipping between two low frequencies."""
    buzzer.duty_u16(32768)
    t = 0
    while t < ms:
        buzzer.freq(low)
        await asyncio.sleep_ms(15)
        buzzer.freq(high)
        await asyncio.sleep_ms(15)
        t += 30
    buzzer.duty_u16(0)


async def happy():
    # quick rising arpeggio (C5, E5, G5, C6)
    await tone(523, 90, 20)
    await tone(659, 90, 20)
    await tone(784, 90, 20)
    await tone(1047, 250)


async def annoyed():
    # two flat, slightly falling "hmph" beeps
    await tone(440, 160, 120)
    await tone(330, 280)


async def mad():
    # five low harsh growls
    for _ in range(5):
        await growl(300)
        await asyncio.sleep_ms(80)

async def sad():
    # Smooth descending slide (sigh / sad trombone effect)
    buzzer.duty_u16(32768)
    for freq in range(400, 220, -15):
        buzzer.freq(freq)
        await asyncio.sleep_ms(30)
    buzzer.duty_u16(0)


SOUNDS = {"happy": happy, "annoyed": annoyed, "mad": mad, "sad": sad}
current = None


async def _run(sound):
    try:
        await sound()
    finally:
        buzzer.duty_u16(0)          # always silence the buzzer, even if cancelled


def play(name):
    """Start a sound without blocking. A new sound cancels the one still playing."""
    global current
    if current is not None:
        current.cancel()
    current = asyncio.create_task(_run(SOUNDS[name]))


# Stand-alone demo: only runs when you run buzzer.py directly, not when main.py imports it.
if __name__ == "__main__":
    async def main():
        play("happy")
        await asyncio.sleep(2)
        play("sad")
        await asyncio.sleep(2)
        play("mad")
        await asyncio.sleep(5)

    asyncio.run(main())


