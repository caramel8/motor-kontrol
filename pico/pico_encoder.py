from machine import Pin, UART
import time

# UART0 (GP0 - RX, GP1 - TX) - Pico'dan Pi'ye veri gönderilecek
uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))

# Motor 1 encoder pinleri (örnek: GP2 ve GP3)
encoder1_a = Pin(2, Pin.IN)
encoder1_b = Pin(3, Pin.IN)

# Motor 2 encoder pinleri (örnek: GP4 ve GP5)
encoder2_a = Pin(4, Pin.IN)
encoder2_b = Pin(5, Pin.IN)

position1 = 0
position2 = 0

last_state1 = encoder1_a.value()
last_state2 = encoder2_a.value()

while True:
    # Motor 1
    current_state1 = encoder1_a.value()
    if current_state1 != last_state1:
        if encoder1_b.value() != current_state1:
            position1 += 1
        else:
            position1 -= 1
        last_state1 = current_state1

    # Motor 2
    current_state2 = encoder2_a.value()
    if current_state2 != last_state2:
        if encoder2_b.value() != current_state2:
            position2 += 1
        else:
            position2 -= 1
        last_state2 = current_state2

    # Pi'ye veri gönder
    uart.write(f"{position1},{position2}\n")
    time.sleep(0.01)
