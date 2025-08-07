import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import time
import serial

class PDController:
    def __init__(self, kp, kd):
        self.kp = kp
        self.kd = kd
        self.prev_error = 0

    def compute(self, setpoint, measured, dt):
        error = setpoint - measured
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = self.kp * error + self.kd * derivative
        self.prev_error = error
        return output

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.subscription = self.create_subscription(
            Int32,
            'encoder_position',
            self.listener_callback,
            10)
        
        self.controller = PDController(kp=1.0, kd=0.1)
        self.setpoint = 100  # Hedef pozisyon
        self.last_time = self.get_clock().now()

        # Pico'ya sinyal gönderilecek seri port
        self.ser = serial.Serial('/dev/ttyAC1', 115200)  # Port ismini kontrol et

    def listener_callback(self, msg):
        current_value = msg.data
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        control_signal = self.controller.compute(self.setpoint, current_value, dt)

        # Kontrol sinyalini sınırlayalım (örnek: -100 ile 100 arası)
        control_signal = max(min(int(control_signal), 100), -100)

        # Pico'ya gönder (örnek: "50\n", "-20\n")
        self.ser.write(f"{control_signal}\n".encode())

        self.get_logger().info(f'Sent: {control_signal}, Measured: {current_value}')

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
