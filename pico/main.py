# === main.py (USB-CDC bridge'e uygun sürüm) ===
from machine import Pin, PWM
import micropython, time, sys, uselect

micropython.alloc_emergency_exception_buf(128)

# ----------------- ENCODER PINS & POWER -----------------
enc1_a = Pin(11, Pin.IN, Pin.PULL_UP)
enc1_b = Pin(10, Pin.IN, Pin.PULL_UP)
enc2_a = Pin(13, Pin.IN, Pin.PULL_UP)
enc2_b = Pin(12, Pin.IN, Pin.PULL_UP)

# Harici 4.7k–10k pull-up önerilir (open-collector için şart)
encoder_vcc = Pin(14, Pin.OUT)
encoder_vcc.high()

# ----------------- ENCODER STATE -----------------
pos1 = 0
pos2 = 0
prev1 = (enc1_a.value() << 1) | enc1_b.value()
prev2 = (enc2_a.value() << 1) | enc2_b.value()

# Quadrature komşu-geçiş tablosu
trans = {
    0b00: {0b01:+1, 0b10:-1},
    0b01: {0b11:+1, 0b00:-1},
    0b11: {0b10:+1, 0b01:-1},
    0b10: {0b00:+1, 0b11:-1},
}

def enc1_isr(pin):
    global pos1, prev1
    curr = (enc1_a.value() << 1) | enc1_b.value()
    pos1 += trans.get(prev1, {}).get(curr, 0)
    prev1 = curr

def enc2_isr(pin):
    global pos2, prev2
    curr = (enc2_a.value() << 1) | enc2_b.value()
    pos2 += trans.get(prev2, {}).get(curr, 0)
    prev2 = curr

# 4x sayım: A ve B hem rising hem falling
enc1_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=enc1_isr)
enc1_b.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=enc1_isr)
enc2_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=enc2_isr)
enc2_b.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=enc2_isr)

# ----------------- PWM OUTPUTS -----------------
motor1_pwm = PWM(Pin(9))   # pinleri donanımına göre değiştir
motor2_pwm = PWM(Pin(4))
motor1_pwm.freq(1000)
motor2_pwm.freq(1000)

def pwm_from_control(percent):
    # percent: -100 .. +100
    middle = 32768
    scale = int(percent * 32768 / 100)
    val = middle + scale
    if val < 0: val = 0
    if val > 65535: val = 65535
    return val

# ----------------- USB-CDC I/O -----------------
# USB-CDC stdin'i bloklamadan kontrol etmek için poll kullan
poll = uselect.poll()
poll.register(sys.stdin, uselect.POLLIN)

last = time.ticks_ms()

while True:
    # --- USB-CDC'den komut oku: "m1,m2" ---
    if poll.poll(0):  # non-blocking
        line = sys.stdin.readline()
        if line:
            try:
                line = line.strip()  # \r\n temizler
                if (',' in line) and line:
                    m1s, m2s = line.split(',', 1)
                    c1 = int(m1s)
                    c2 = int(m2s)
                    # saturate
                    if c1 < -100: c1 = -100
                    if c1 > 100:  c1 = 100
                    if c2 < -100: c2 = -100
                    if c2 > 100:  c2 = 100
                    motor1_pwm.duty_u16(pwm_from_control(c1))
                    motor2_pwm.duty_u16(pwm_from_control(c2))
            except Exception:
                # format hatalarını sessizce yut (çıktıyı kirletmemek için)
                pass

    # --- 100 Hz encoder raporu (USB-CDC): sadece "pos1,pos2" ---
    now = time.ticks_ms()
    if time.ticks_diff(now, last) >= 10:
        print(pos1, pos2, sep=',')
        last = now

    time.sleep_ms(1)
