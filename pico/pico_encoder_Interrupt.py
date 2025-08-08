# === encoder_usb.py ===
from machine import Pin
import micropython, time
import sys

micropython.alloc_emergency_exception_buf(100)

# --- PINLER (senin verdiğin) ---
enc1_a = Pin(26, Pin.IN, Pin.PULL_UP)
enc1_b = Pin(20, Pin.IN, Pin.PULL_UP)
enc2_a = Pin(2,  Pin.IN, Pin.PULL_UP)
enc2_b = Pin(3,  Pin.IN, Pin.PULL_UP)
# (Haricî 4.7k–10k pull-up önerilir; open-collector için şart gibi)
encoder_vcc = Pin(0, Pin.OUT)
encoder_vcc.high()

pos1 = 0
pos2 = 0
prev1 = (enc1_a.value() << 1) | enc1_b.value()
prev2 = (enc2_a.value() << 1) | enc2_b.value()

# Komşu geçiş tablosu (quadrature)
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

# 4×: A ve B, yükselen+düşen
enc1_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=enc1_isr)
enc1_b.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=enc1_isr)
enc2_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=enc2_isr)
enc2_b.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=enc2_isr)

# 100 Hz raporla (USB-CDC: print)
last = time.ticks_ms()

while True:
    now = time.ticks_ms()
    if time.ticks_diff(now, last) >= 10:
        # Sadece iki sayı ve arada virgül
        print(pos1, pos2, sep=',')
        last = now
    time.sleep_ms(1)
