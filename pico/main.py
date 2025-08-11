# === main.py ===
from machine import Pin, PWM, UART
import micropython, time

micropython.alloc_emergency_exception_buf(100)

# ----------------- ENCODER PINS & POWER -----------------
enc1_a = Pin(11, Pin.IN, Pin.PULL_UP)
enc1_b = Pin(10, Pin.IN, Pin.PULL_UP)
enc2_a = Pin(13, Pin.IN, Pin.PULL_UP)
enc2_b = Pin(12, Pin.IN, Pin.PULL_UP)

# Harici pull-up önerilir (4.7k–10k). Open-collector ise şart.
encoder_vcc = Pin(14, Pin.OUT)
encoder_vcc.high()

# ----------------- ENCODER STATE -----------------
pos1 = 0
pos2 = 0
prev1 = (enc1_a.value() << 1) | enc1_b.value()
prev2 = (enc2_a.value() << 1) | enc2_b.value()

# Quadrature komşu-geçiş tablosu (hatalı sıçramaları 0 say)
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

# 4x sayım: A ve B hem yükselen hem düşen kenarda
enc1_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=enc1_isr)
enc1_b.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=enc1_isr)
enc2_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=enc2_isr)
enc2_b.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=enc2_isr)

# ----------------- UART (CONTROL IN) -----------------
# UART0: GP1=RX, GP0=TX (Pico default)
uart = UART(0, baudrate=115200, rx=Pin(1), tx=Pin(0))

# ----------------- PWM OUTPUTS -----------------
# Örnek PWM pinleri (gerekirse değiştir)
motor1_pwm = PWM(Pin(9))
motor2_pwm = PWM(Pin(4))
motor1_pwm.freq(1000)
motor2_pwm.freq(1000)

def pwm_from_control(percent):
    """
    percent: -100 .. +100 arası kontrol (%)
    0 -> 32768 (orta), -100 -> 0, +100 -> 65535
    """
    middle = 32768
    scale = int(percent * 32768 / 100)
    val = middle + scale
    if val < 0:
        val = 0
    elif val > 65535:
        val = 65535
    return val

# ----------------- MAIN LOOP -----------------
last = time.ticks_ms()

while True:
    # --- UART komutu oku: "m1,m2" ---
    if uart.any():
        line = uart.readline()
        if line:
            try:
                line = line.decode().strip()
                if line:
                    m1_str, m2_str = line.split(",")
                    c1 = int(m1_str)
                    c2 = int(m2_str)
                    # opsiyonel: aralık sıkıştır
                    if c1 < -100: c1 = -100
                    if c1 > 100:  c1 = 100
                    if c2 < -100: c2 = -100
                    if c2 > 100:  c2 = 100
                    motor1_pwm.duty_u16(pwm_from_control(c1))
                    motor2_pwm.duty_u16(pwm_from_control(c2))
            except Exception:
                # Parser hatalarını sessizce yut (USB çıktısını kirletme)
                pass

    # --- 100 Hz encoder raporu (USB-CDC): sadece "pos1,pos2" ---
    now = time.ticks_ms()
    if time.ticks_diff(now, last) >= 10:
        # ROS tarafı beklediği format: "int,int\n"
        print(pos1, pos2, sep=',')
        last = now

    # Küçük uyku: CPU rahatlasın
    time.sleep_ms(1)
