# === main.py (USB-CDC + Encoder + L298N IN1..IN4, flush'lı) ===
from machine import Pin, PWM
import micropython, time, sys, uselect

micropython.alloc_emergency_exception_buf(128)

# ----------------- ENCODER PINS & POWER -----------------
enc2_b = Pin(11, Pin.IN, Pin.PULL_UP)
enc2_a = Pin(10, Pin.IN, Pin.PULL_UP)
enc1_b = Pin(13, Pin.IN, Pin.PULL_UP)
enc1_a = Pin(12, Pin.IN, Pin.PULL_UP)

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

# ----------------- L298N DIRECTION PINS -----------------
# IN1, IN2, IN3, IN4 -> GP8, GP7, GP6, GP5
IN1 = Pin(7, Pin.OUT);  IN1.low()
IN2 = Pin(8, Pin.OUT);  IN2.low()
IN3 = Pin(5, Pin.OUT);  IN3.low()
IN4 = Pin(6, Pin.OUT);  IN4.low()

# ----------------- PWM OUTPUTS (ENA/ENB) -----------------
# ENA -> GP9, ENB -> GP4 (PWM)
motor1_pwm = PWM(Pin(9))   # Motor1 hız (ENA)
motor2_pwm = PWM(Pin(4))   # Motor2 hız (ENB)
motor1_pwm.freq(1000)
motor2_pwm.freq(1000)

def duty_from_percent(p):
    # p: 0..100
    if p < 0: p = 0
    if p > 100: p = 100
    return int(p * 65535 / 100)

def set_motor(motor_idx, percent):
    """percent: -100..100 ; yön IN pinleriyle, hız PWM ile"""
    if percent > 100:  percent = 100
    if percent < -100: percent = -100
    speed = duty_from_percent(abs(percent))

    if motor_idx == 1:
        # Yön
        if percent > 0:
            IN1.high(); IN2.low()     # ileri
        elif percent < 0:
            IN1.low();  IN2.high()    # geri
        else:
            IN1.low();  IN2.low()     # coast
        # Hız
        motor1_pwm.duty_u16(speed if percent != 0 else 0)

    elif motor_idx == 2:
        if percent > 0:
            IN3.high(); IN4.low()
        elif percent < 0:
            IN3.low();  IN4.high()
        else:
            IN3.low();  IN4.low()
        motor2_pwm.duty_u16(speed if percent != 0 else 0)

# ----------------- USB-CDC I/O -----------------
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
                    c1 = int(m1s); c2 = int(m2s)
                    set_motor(1, c1)
                    set_motor(2, c2)
            except Exception:
                # format hatalarını sessizce yut
                pass

    # --- 100 Hz encoder raporu (USB-CDC): sadece "pos1,pos2" ---
    now = time.ticks_ms()
    if time.ticks_diff(now, last) >= 10:
        try:
            # flush'lı yazım
            sys.stdout.write(f"{pos1},{pos2}\r\n")
            if hasattr(sys.stdout, "flush"):
                sys.stdout.flush()
        except Exception:
            pass
        last = now

    time.sleep_ms(1)

