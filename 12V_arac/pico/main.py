# === main.py (USB-CDC + 4x Encoder + 4x DC motor, 2-PWM/motor, flush'lı) ===
from machine import Pin, PWM
import micropython, time, sys, uselect

micropython.alloc_emergency_exception_buf(128)

# ----------------- ENCODER PINS (A,B) -----------------
# M1: A->GP1,  B->GP0
enc1_a = Pin(1,  Pin.IN)
enc1_b = Pin(0,  Pin.IN)

# M2: A->GP14, B->GP15
enc2_a = Pin(14, Pin.IN)
enc2_b = Pin(15, Pin.IN)

# M3: A->GP26, B->GP27
enc3_a = Pin(26, Pin.IN)
enc3_b = Pin(27, Pin.IN)

# M4: A->GP17, B->GP16
enc4_a = Pin(17, Pin.IN)
enc4_b = Pin(16, Pin.IN)

# ----------------- ENCODER STATE -----------------
pos1 = 0; pos2 = 0; pos3 = 0; pos4 = 0
prev1 = (enc1_a.value() << 1) | enc1_b.value()
prev2 = (enc2_a.value() << 1) | enc2_b.value()
prev3 = (enc3_a.value() << 1) | enc3_b.value()
prev4 = (enc4_a.value() << 1) | enc4_b.value()

# Komşu-geçiş tablosu (quadrature 4x)
trans = {
    0b00: {0b01:+1, 0b10:-1},
    0b01: {0b11:+1, 0b00:-1},
    0b11: {0b10:+1, 0b01:-1},
    0b10: {0b00:+1, 0b11:-1},
}

def enc_isr_generic(enc_a, enc_b, prev_name, pos_name):
    globals_dict = globals()
    curr = (enc_a.value() << 1) | enc_b.value()
    prev = globals_dict[prev_name]
    delta = trans.get(prev, {}).get(curr, 0)
    if delta:
        globals_dict[pos_name] += delta
    globals_dict[prev_name] = curr

def enc1_isr(pin): enc_isr_generic(enc1_a, enc1_b, 'prev1', 'pos1')
def enc2_isr(pin): enc_isr_generic(enc2_a, enc2_b, 'prev2', 'pos2')
def enc3_isr(pin): enc_isr_generic(enc3_a, enc3_b, 'prev3', 'pos3')
def enc4_isr(pin): enc_isr_generic(enc4_a, enc4_b, 'prev4', 'pos4')

# 4x sayım için A ve B hem rising hem falling
for p in (enc1_a, enc1_b): p.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=enc1_isr)
for p in (enc2_a, enc2_b): p.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=enc2_isr)
for p in (enc3_a, enc3_b): p.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=enc3_isr)
for p in (enc4_a, enc4_b): p.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=enc4_isr)

# ----------------- MOTOR PWM PINS (A,B) -----------------
# M1 PWM: GP4, GP5
m1_pwm_a = PWM(Pin(4))
m1_pwm_b = PWM(Pin(5))

# M2 PWM: GP12, GP13
m2_pwm_a = PWM(Pin(12))
m2_pwm_b = PWM(Pin(13))

# M3 PWM: GP22, GP21
m3_pwm_a = PWM(Pin(22))
m3_pwm_b = PWM(Pin(21))

# M4 PWM: GP19, GP18
m4_pwm_a = PWM(Pin(19))
m4_pwm_b = PWM(Pin(18))

ALL_PWMS = (m1_pwm_a, m1_pwm_b, m2_pwm_a, m2_pwm_b, m3_pwm_a, m3_pwm_b, m4_pwm_a, m4_pwm_b)
for p in ALL_PWMS:
    p.freq(1000)
    p.duty_u16(0)

def duty_from_percent(p):
    # p: 0..100
    if p < 0: p = 0
    if p > 100: p = 100
    return int(p * 65535 / 100)

def set_motor_pair(pwm_a: PWM, pwm_b: PWM, percent: int):
    """
    İki PWM ile sürüş:
      percent > 0  => A = duty, B = 0  (ileri)
      percent < 0  => A = 0,    B = duty (geri)
      percent == 0 => her ikisi 0 (coast)
    """
    if percent > 100:  percent = 100
    if percent < -100: percent = -100
    if percent > 0:
        pwm_a.duty_u16(duty_from_percent(percent))
        pwm_b.duty_u16(0)
    elif percent < 0:
        pwm_a.duty_u16(0)
        pwm_b.duty_u16(duty_from_percent(-percent))
    else:
        pwm_a.duty_u16(0)
        pwm_b.duty_u16(0)

def set_motor(motor_idx: int, percent: int):
    if motor_idx == 1:
        set_motor_pair(m1_pwm_a, m1_pwm_b, percent)
    elif motor_idx == 2:
        set_motor_pair(m2_pwm_a, m2_pwm_b, percent)
    elif motor_idx == 3:
        set_motor_pair(m3_pwm_a, m3_pwm_b, percent)
    elif motor_idx == 4:
        set_motor_pair(m4_pwm_a, m4_pwm_b, percent)

# ----------------- USB-CDC I/O -----------------
poll = uselect.poll()
poll.register(sys.stdin, uselect.POLLIN)

last = time.ticks_ms()

while True:
    # --- USB-CDC'den komut oku: "m1,m2,m3,m4" ---
    if poll.poll(0):  # non-blocking
        line = sys.stdin.readline()
        if line:
            try:
                line = line.strip()
                if line and (line.count(',') >= 3):
                    s1, s2, s3, s4 = line.split(',', 3)
                    c1 = int(s1); c2 = int(s2); c3 = int(s3); c4 = int(s4)
                    set_motor(1, c1)
                    set_motor(2, c2)
                    set_motor(3, c3)
                    set_motor(4, c4)
            except Exception:
                # format hatalarını sessizce yut
                pass

    # --- 100 Hz encoder raporu: "pos1,pos2,pos3,pos4" ---
    now = time.ticks_ms()
    if time.ticks_diff(now, last) >= 10:
        try:
            sys.stdout.write(f"{pos1},{pos2},{pos3},{pos4}\r\n")
            if hasattr(sys.stdout, "flush"):
                sys.stdout.flush()
        except Exception:
            pass
        last = now

    time.sleep_ms(1)

