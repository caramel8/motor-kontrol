from machine import Pin, PWM, UART
import time

# UART0 (GP0 - RX, GP1 - TX)
uart = UART(0, baudrate=115200, rx=Pin(1), tx=Pin(0))

# İki motorun PWM pinleri (örnek: GP15 ve GP14)
motor1_pwm = PWM(Pin(15))
motor2_pwm = PWM(Pin(14))
motor1_pwm.freq(1000)
motor2_pwm.freq(1000)

def pwm_from_control(control_signal):
    middle = 32768
    scale = int(control_signal * 32768 / 100)
    return max(0, min(65535, middle + scale))

while True:
    if uart.any():
        try:
            line = uart.readline().decode().strip()
            motor1_str, motor2_str = line.split(",")
            control1 = int(motor1_str)
            control2 = int(motor2_str)
            motor1_pwm.duty_u16(pwm_from_control(control1))
            motor2_pwm.duty_u16(pwm_from_control(control2))
            print(f"Motor1: {control1}, Motor2: {control2}")
        except Exception as e:
            print("Error:", e)
    time.sleep(0.01)
