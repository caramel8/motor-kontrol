# === main.py ===
from machine import Pin, PWM, UART
import micropython, time

# --- Güvenlik: ISR'larda hata mesajı için acil buffer ---
micropython.alloc_emergency_exception_buf(100)

# --- UART0: GP0 (TX), GP1 (RX) ---
uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))

# --- Encoder besleme (varsa) ---
encoder_vcc = Pin(14, Pin.OUT)
encoder_vcc.high()

# --- Encoder pinleri (pull-up'lı) ---
enc1_a = Pin(11, Pin.IN, Pin.PULL_UP)
enc1_b = Pin(10, Pin.IN, Pin.PULL_UP)
enc2_a = Pin(13, Pin.IN, Pin.PULL_UP)
enc2_b = Pin(12, Pin.IN, Pin.PULL_UP)

# --- Quadrature sayıcı değişkenleri ---
pos1 = 0
pos2 = 0
prev1 = (enc1_a.value() << 1) | enc1_b.value()
prev2 = (enc2_a.value() << 1) | enc2_b.value()

# Komşu geçiş tablosu (yalnızca geçerli geçişler puanlanır)
_trans = {
    0b00: {0b01:+1, 0b10:-1},
    0b01: {0b11:+1, 0b00:-1},
    0b11: {0b10:+1, 0b01:-1},
    0b10: {0b00:+1, 0b11:-1},
}

def _enc1_isr(pin):
    global pos1, prev1
    curr = (enc1_a.value() << 1) | enc1_b.value()
    pos1 += _trans.get(prev1, {}).get(curr, 0)
    prev1 = curr

def _enc2_isr(pin):
    global pos2, prev2
    curr = (enc2_a.value() << 1) | enc2_b.value()
    pos2 += _trans.get(prev2, {}).get(curr, 0)
    prev2 = curr

# 4× çözünürlük: A ve B, yükselen + düşen kenarlar
enc1_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=_enc1_isr)
enc1_b.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=_enc1_isr)
enc2_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=_enc2_isr)
enc2_b.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=_enc2_isr)

# --- PWM çıkışları (1 kHz) ---
motor1_pwm = PWM(Pin(9))
motor2_pwm = PWM(Pin(4))
motor1_pwm.freq(1000)
motor2_pwm.freq(1000)

def pwm_from_control(control_percent):
    """
    control_percent: -100 .. +100 (int/float)
    Tek PWM kullanılan sürücüler için 'orta=dur' yaklaşımı:
    0% -> duty 32768 (orta), +100% -> 65535, -100% -> 0
    """
    middle = 32768
    scale = int(max(-100, min(100, control_percent)) * 32768 / 100)
    return max(0, min(65535, middle + scale))

# --- Ana döngü: her 10 ms'de raporla, arada komut oku ---
_last = time.ticks_ms()
while True:
    # 1) Raspberry Pi'den komut oku (örn. "20,-15\n")
    if uart.any():
        line = uart.readline()
        if line:
            try:
                line = line.decode().strip()
                if line:  # boş satır değilse
                    s1, s2 = line.split(',')
                    c1 = int(float(s1))
                    c2 = int(float(s2))
                    motor1_pwm.duty_u16(pwm_from_control(c1))
                    motor2_pwm.duty_u16(pwm_from_control(c2))
            except Exception as e:
                # Hatalı satırı yut, döngüye devam et
                # (İstersen uart.write ile hata döndürebiliriz)
                pass

    # 2) 100 Hz pozisyon raporu (UART0 TX ile Pi'ye)
    now = time.ticks_ms()
    if time.ticks_diff(now, _last) >= 10:
        try:
            uart.write("{}{},{}{}\n".format('', pos1, pos2, ''))
        except Exception:
            pass
        _last = now

    time.sleep_ms(1)
