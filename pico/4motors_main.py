# === main_4motors.py (USB-CDC + 4x Encoder + 2x L298N, 4 motor) ===
from machine import Pin, PWM
import micropython, time, sys, uselect

micropython.alloc_emergency_exception_buf(128)

# -------------------------------------------------------------------
# PIN HARİTASI (ENCODERLER)
# -------------------------------------------------------------------
# Encoder 1 (Motor 1)
ENC1_A_PIN = 13
ENC1_B_PIN = 12
# Encoder 2 (Motor 2)
ENC2_A_PIN = 10
ENC2_B_PIN = 11
# Encoder 3 (Motor 3)
ENC3_A_PIN = 3
ENC3_B_PIN = 2
# Encoder 4 (Motor 4)
ENC4_A_PIN = 1
ENC4_B_PIN = 0

# -------------------------------------------------------------------
# ENCODER BESLEME (GPIO ile 3.3V enable hatları)
# Uyarı: GPIO'dan akım sınırlıdır; düşük akım modüller için uygundur.
# -------------------------------------------------------------------
encoder_vcc  = Pin(14, Pin.OUT); encoder_vcc.high()   # Grup-1 (örn. Enc1-2)
encoder_vcc2 = Pin(26, Pin.OUT); encoder_vcc2.high()  # Grup-2 (örn. Enc3)
encoder_vcc3 = Pin(27, Pin.OUT); encoder_vcc3.high()  # Grup-3 (örn. Enc4)

def set_encoder_power(idx, on: bool):
    """idx: 1 -> GP14, 2 -> GP26, 3 -> GP27"""
    lvl = 1 if on else 0
    if idx == 1:
        encoder_vcc.value(lvl)
    elif idx == 2:
        encoder_vcc2.value(lvl)
    elif idx == 3:
        encoder_vcc3.value(lvl)

# -------------------------------------------------------------------
# L298N BAĞLANTILARI (Motor1..4) — IN ve EN(PWM)
# -------------------------------------------------------------------
# L298N #1 (Motor1 & Motor2)
M1_IN1_PIN = 8    # Motor1 yön
M1_IN2_PIN = 7
M1_EN_PIN  = 9    # Motor1 PWM (ENA)
M2_IN1_PIN = 5    # Motor2 yön
M2_IN2_PIN = 6
M2_EN_PIN  = 4    # Motor2 PWM (ENB)

# L298N #2 (Motor3 & Motor4)
M4_IN1_PIN = 17   # Motor3 yön
M4_IN2_PIN = 16
M4_EN_PIN  = 18   # Motor3 PWM
M3_IN1_PIN = 20   # Motor4 yön
M3_IN2_PIN = 21
M3_EN_PIN  = 19   # Motor4 PWM

PWM_FREQ_HZ = 1000

# -------------------------------------------------------------------
# ENCODER KURULUMU
# -------------------------------------------------------------------
enc_pins = [
    (ENC1_A_PIN, ENC1_B_PIN),
    (ENC2_A_PIN, ENC2_B_PIN),
    (ENC3_A_PIN, ENC3_B_PIN),
    (ENC4_A_PIN, ENC4_B_PIN),
]

encoders = []
for a_pin, b_pin in enc_pins:
    a = Pin(a_pin, Pin.IN, Pin.PULL_UP)
    b = Pin(b_pin, Pin.IN, Pin.PULL_UP)
    encoders.append((a, b))

# Sayaçlar
pos = [0, 0, 0, 0]
prev = []
for i in range(4):
    a, b = encoders[i]
    prev.append((a.value() << 1) | b.value())

# Quadrature komşu-geçiş tablosu (hatalı sıçramaları 0 sayar)
trans = {
    0b00: {0b01:+1, 0b10:-1},
    0b01: {0b11:+1, 0b00:-1},
    0b11: {0b10:+1, 0b01:-1},
    0b10: {0b00:+1, 0b11:-1},
}

def make_isr(idx):
    def _isr(pin):
        a, b = encoders[idx]
        curr = (a.value() << 1) | b.value()
        pos[idx] += trans.get(prev[idx], {}).get(curr, 0)
        prev[idx] = curr
    return _isr

# 4x sayım (A ve B rising+falling)
for i in range(4):
    a, b = encoders[i]
    isr = make_isr(i)
    a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=isr)
    b.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=isr)

# -------------------------------------------------------------------
# MOTOR SÜRÜCÜ (L298N) — YÖN + PWM
# -------------------------------------------------------------------
class Motor:
    def __init__(self, in1_pin, in2_pin, en_pin, pwm_freq=PWM_FREQ_HZ):
        self.in1 = Pin(in1_pin, Pin.OUT); self.in1.low()
        self.in2 = Pin(in2_pin, Pin.OUT); self.in2.low()
        self.en  = PWM(Pin(en_pin))
        self.en.freq(pwm_freq)
        self.set_percent(0)

    @staticmethod
    def duty_from_percent(p):
        if p < 0: p = 0
        if p > 100: p = 100
        return int(p * 65535 / 100)

    def set_percent(self, percent):  # -100..100
        if percent > 100:  percent = 100
        if percent < -100: percent = -100
        speed = Motor.duty_from_percent(abs(percent))

        if percent > 0:
            self.in1.high(); self.in2.low()
        elif percent < 0:
            self.in1.low();  self.in2.high()
        else:
            # coast; fren istersen: self.in1.high(); self.in2.high()
            self.in1.low();  self.in2.low()
            speed = 0

        self.en.duty_u16(speed)

motors = [
    Motor(M1_IN1_PIN, M1_IN2_PIN, M1_EN_PIN),  # Motor 1
    Motor(M2_IN1_PIN, M2_IN2_PIN, M2_EN_PIN),  # Motor 2
    Motor(M3_IN1_PIN, M3_IN2_PIN, M3_EN_PIN),  # Motor 3
    Motor(M4_IN1_PIN, M4_IN2_PIN, M4_EN_PIN),  # Motor 4
]

def set_motor(idx, percent):
    motors[idx-1].set_percent(percent)

# -------------------------------------------------------------------
# USB-CDC I/O
# -------------------------------------------------------------------
poll = uselect.poll()
poll.register(sys.stdin, uselect.POLLIN)

last = time.ticks_ms()

while True:
    # Komut: "m1,m2,m3,m4"  örn: "30,-30,0,50"
    if poll.poll(0):
        line = sys.stdin.readline()
        if line:
            try:
                s = line.strip()
                if s and (',' in s):
                    parts = s.split(',')
                    vals = [int(x) for x in parts[:4]]
                    while len(vals) < 4:
                        vals.append(0)
                    set_motor(1, vals[0])
                    set_motor(2, vals[1])
                    set_motor(3, vals[2])
                    set_motor(4, vals[3])
            except Exception:
                pass  # format hatalarını sessizce yut

    # 100 Hz encoder raporu: "p1,p2,p3,p4"
    now = time.ticks_ms()
    if time.ticks_diff(now, last) >= 10:
        try:
            sys.stdout.write(f"{pos[0]},{pos[1]},{pos[2]},{pos[3]}\r\n")
            if hasattr(sys.stdout, "flush"):
                sys.stdout.flush()
        except Exception:
            pass
        last = now

    time.sleep_ms(1)

