from machine import Pin, PWM, UART
import time

# UART0: Pi'den veri almak için (GP0 - RX, GP1 - TX)
uart = UART(0, baudrate=115200, rx=Pin(1), tx=Pin(0))

# PWM pin tanımla (örneğin GP15)
pwm_pin = PWM(Pin(15))
pwm_pin.freq(1000)  # 1kHz PWM

# PWM duty cycle sınırları (0 - 65535)
def pwm_value_from_control(control_signal):
    # -100 to 100 --> 3276 to 62259 (orta 50%)
    middle = 32768
    scale = int(control_signal * 32768 / 100)
    return max(0, min(65535, middle + scale))

while True:
    if uart.any():
        try:
            line = uart.readline().decode().strip()
            control = int(line)
            pwm_val = pwm_value_from_control(control)
            pwm_pin.duty_u16(pwm_val)
            print(f"Received: {control}, PWM: {pwm_val}")
        except Exception as e:
            print("Error:", e)
    time.sleep(0.01)
