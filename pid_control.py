# pid_control.py

class PDController:
    def __init__(self, kp, kd):
        self.kp = kp
        self.kd = kd
        self.prev_error = 0

    def compute(self, setpoint, measured_value, dt):
        error = setpoint - measured_value
        derivative = (error - self.prev_error) / dt if dt > 0 else 0

        output = self.kp * error + self.kd * derivative
        self.prev_error = error

        return output

# Test amaçlı örnek kullanım:
if __name__ == "__main__":
    import time
    pd = PDController(kp=1.0, kd=0.1)

    setpoint = 100  # hedef hız örneği
    current_value = 0
    dt = 0.1  # zaman aralığı (100ms)

    for _ in range(20):
        control = pd.compute(setpoint, current_value, dt)
        current_value += control * dt  # basit modelleme
        print(f"Control: {control:.2f}, Measured: {current_value:.2f}")
        time.sleep(dt)
