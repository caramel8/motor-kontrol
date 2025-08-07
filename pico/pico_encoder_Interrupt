from machine import Pin, UART
import micropython, time

# (1) IRQ içinde hata olursa kilitlenmesin diye acil hata tamponu ayır
micropython.alloc_emergency_exception_buf(100)

# (2) UART: Pi'ye pozisyonları göndermek için
uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))

# (3) Encoder pinleri (PULL_UP çoğu enkoderde gerekli)
enc1_a = Pin(2, Pin.IN, Pin.PULL_UP)
enc1_b = Pin(3, Pin.IN, Pin.PULL_UP)
enc2_a = Pin(4, Pin.IN, Pin.PULL_UP)
enc2_b = Pin(5, Pin.IN, Pin.PULL_UP)

# (4) Pozisyon sayaçları ve önceki 2-bit durumlar
pos1 = 0
pos2 = 0
prev1 = (enc1_a.value() << 1) | enc1_b.value()   # 0b00..0b11 (0..3)
prev2 = (enc2_a.value() << 1) | enc2_b.value()

# (5) Geçiş tablosu (prev -> curr): +1 ileri, -1 geri, 0 geçersiz/atlama
# Sadece komşu geçişler puanlanır:
# ileri: 00->01->11->10->00  (+1)
# geri : 00->10->11->01->00  (-1)
trans = {
    0b00: {0b01:+1, 0b10:-1},
    0b01: {0b11:+1, 0b00:-1},
    0b11: {0b10:+1, 0b01:-1},
    0b10: {0b00:+1, 0b11:-1},
}

# (6) IRQ handler'ları: A VEYA B kenarında çağrılır (4×)
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

# (7) Her iki pin için de yükselen + düşen kenara IRQ bağla → 4× sayım
enc1_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=enc1_isr)
enc1_b.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=enc1_isr)
enc2_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=enc2_isr)
enc2_b.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=enc2_isr)

# (8) Basit raporlama döngüsü (IRQ sayıyor; burada sadece göndermek var)
while True:
    # her 10 ms' de pozisyonları gönder
    try:
        uart.write(f"{pos1},{pos2}\n")
    except:
        pass
    time.sleep_ms(10)
