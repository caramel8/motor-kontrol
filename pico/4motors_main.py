# === main_4motors.py ===
from machine import Pin, PWM
import micropython, time, sys, uselect

micropython.alloc_emergency_exception_buf(256)

# =========================
# KULLANICI AYARLARI
# =========================
PWM_FREQ_HZ = 20000       # 20 kHz: sessiz ve pürüzsüz
REPORT_PERIOD_MS = 10     # ~100 Hz rapor
CONTROL_DT_MS = 10        # kontrol/slew periyodu
VEL_ALPHA = 0.2           # hız filtresi (EMA)
MAX_DU_PER_TICK = 5.0     # her kontrol adımında max değişim (%)

# 4 motor için pin eşleşmeleri (L298N: INx/INy = yön, ENx = PWM)
MOTORS = [
    # M1
    dict(DIR_A=6,  DIR_B=7,  PWM_PIN=8,  ENC_A=11, ENC_B=10),
    # M2
    dict(DIR_A=4,  DIR_B=5,  PWM_PIN=9,  ENC_A=13, ENC_B=12),
    # M3  (2. L298N kullanıyorsan buraya o kartın pinlerini yaz)
    dict(DIR_A=18, DIR_B=19, PWM_PIN=20, ENC_A=None, ENC_B=None),  # enkoder yoksa None
    # M4
    dict(DIR_A=14, DIR_B=15, PWM_PIN=16, ENC_A=None, ENC_B=None),
]

# Enkoder besleme pini (opsiyonel)
ENC_PWR_PIN = 14  # boşsa mevcut bir 3V3 kullan
try:
    enc_pwr = Pin(ENC_PWR_PIN, Pin.OUT)
    enc_pwr.high()
except Exception:
    pass

# =========================
# GLOBAL DURUMLAR
# =========================
N = 4
pos = [0]*N
prev = [0]*N     # quadrature önceki durumu
vel = [0.0]*N    # ticks/s
last_pos = [0]*N

# Quadrature geçiş tablosu (4x sayım)
trans = {
    0b00: {0b01:+1, 0b10:-1},
    0b01: {0b11:+1, 0b00:-1},
    0b11: {0b10:+1, 0b01:-1},
    0b10: {0b00:+1, 0b11:-1},
}

# =========================
# PIN/ISR KURULUMU
# =========================
# Motor yön ve PWM pinleri
dirs_a = []
dirs_b = []
pwms   = []

# Enkoder pinleri (None olabilir)
enc_a_pins = [None]*N
enc_b_pins = [None]*N

for i, m in enumerate(MOTORS):
    # Yön pinleri
    da = Pin(m["DIR_A"], Pin.OUT); da.value(0)
    db = Pin(m["DIR_B"], Pin.OUT); db.value(0)
    dirs_a.append(da); dirs_b.append(db)

    # PWM pinleri
    p = PWM(Pin(m["PWM_PIN"]))
    p.freq(PWM_FREQ_HZ); p.duty_u16(0)
    pwms.append(p)

    # Enkoder pinleri (varsa)
    ea = m.get("ENC_A", None)
    eb = m.get("ENC_B", None)
    if ea is not None and eb is not None:
        pa = Pin(ea, Pin.IN, Pin.PULL_UP)
        pb = Pin(eb, Pin.IN, Pin.PULL_UP)
        enc_a_pins[i] = pa
        enc_b_pins[i] = pb
        prev[i] = (pa.value() << 1) | pb.value()
    else:
        enc_a_pins[i] = None
        enc_b_pins[i] = None
        prev[i] = 0

# Her motor için ISR üreten fabrika
def make_isr(idx):
    def _isr(pin):
        # Enkoder yoksa çık
        if enc_a_pins[idx] is None: 
            return
        curr = (enc_a_pins[idx].value() << 1) | enc_b_pins[idx].value()
        # Geçerli geçiş varsa +1/-1 ekle
        pos[idx] += trans.get(prev[idx], {}).get(curr, 0)
        prev[idx] = curr
    return _isr

# ISR’leri bağla (enkoder varsa)
for i in range(N):
    if enc_a_pins[i] is not None and enc_b_pins[i] is not None:
        isr = make_isr(i)
        enc_a_pins[i].irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=isr)
        enc_b_pins[i].irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=isr)

# =========================
# MOTOR SÜRME / DURDURMA
# =========================
def set_motor(i, u_percent):  # i: 0..3, u: -100..+100
    if u_percent > 100: u_percent = 100
    if u_percent < -100: u_percent = -100
    duty = int(abs(u_percent) * 65535 / 100)

    if u_percent >= 0:
        dirs_a[i].value(1); dirs_b[i].value(0)
    else:
        dirs_a[i].value(0); dirs_b[i].value(1)
    pwms[i].duty_u16(duty)

def stop_all(brake=False):
    for i in range(N):
        pwms[i].duty_u16(0)
        if brake:
            # L298N kısa devre fren: iki bacağı HIGH
            dirs_a[i].value(1); dirs_b[i].value(1)
        else:
            # coast
            dirs_a[i].value(0); dirs_b[i].value(0)

# =========================
# HIZ TAHMİNİ
# =========================
def update_velocity(dt_s):
    if dt_s <= 0: return
    for i in range(N):
        if enc_a_pins[i] is None:
            vel[i] = 0.0
            last_pos[i] = pos[i]
            continue
        raw = (pos[i] - last_pos[i]) / dt_s
        vel[i] = (1.0 - VEL_ALPHA)*vel[i] + VEL_ALPHA*raw
        last_pos[i] = pos[i]

# =========================
# USB-CDC I/O
# =========================
poll = uselect.poll()
poll.register(sys.stdin, uselect.POLLIN)

last_report = time.ticks_ms()
last_ctrl = time.ticks_ms()

# Slew-limit için komut ve çıkışlar
u_cmd = [0.0]*N
u_out = [0.0]*N

def slew(to_val, cur_val, step):
    if to_val > cur_val + step: return cur_val + step
    if to_val < cur_val - step: return cur_val - step
    return to_val

while True:
    # --- Komut oku ---
    if poll.poll(0):
        line = sys.stdin.readline()
        if line:
            try:
                s = line.strip()
                if not s:
                    pass
                elif s == 'S':
                    stop_all(False); u_cmd = [0.0]*N
                elif s == 'SB':
                    stop_all(True);  u_cmd = [0.0]*N
                elif s.startswith('PWMF '):
                    hz = int(s.split()[1])
                    if 200 <= hz <= 50000:
                        for p in pwms: p.freq(hz)
                else:
                    # "m1,m2,m3,m4"
                    parts = s.split(',')
                    if len(parts) == N:
                        vals = [int(v) for v in parts]
                        for i in range(N):
                            if vals[i] > 100:  vals[i] = 100
                            if vals[i] < -100: vals[i] = -100
                            u_cmd[i] = float(vals[i])
            except Exception:
                pass

    # --- Kontrol & hız hesap ---
    now = time.ticks_ms()
    if time.ticks_diff(now, last_ctrl) >= CONTROL_DT_MS:
        dt_s = time.ticks_diff(now, last_ctrl) / 1000.0
        last_ctrl = now

        update_velocity(dt_s)

        step = MAX_DU_PER_TICK
        for i in range(N):
            u_out[i] = slew(u_cmd[i], u_out[i], step)
            set_motor(i, u_out[i])

    # --- Rapor: pos1,pos2,pos3,pos4 ---
    if time.ticks_diff(now, last_report) >= REPORT_PERIOD_MS:
        # Dilersen hızları da yazdır: print(pos[0],pos[1],pos[2],pos[3], f"{vel[0]:.1f}",..., sep=',')
        print(pos[0], pos[1], pos[2], pos[3], sep=',')
        last_report = now

    time.sleep_ms(1)
