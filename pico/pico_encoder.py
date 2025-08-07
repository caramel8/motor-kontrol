from machine import Pin
import utime

# ENCODER pin tanımları (örnek olarak GP14 ve GP15)
ENCA = Pin(14, Pin.IN)
ENCB = Pin(15, Pin.IN)

# Konum sayacı
position = 0
last_state = ENCA.value()

# Ana döngü
while True:
    current_state = ENCA.value()

    if current_state != last_state:
        if ENCB.value() != current_state:
            position += 1
        else:
            position -= 1

        print(position)  # Bu veriyi Raspberry Pi alacak

    last_state = current_state
    utime.sleep_ms(1)  # Gerekirse hassasiyet için azalt
