import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial
import time

class PDController(Node):
    def __init__(self):
        super().__init__('pd_controller')

        # Hedef pozisyonlar (dilersen değiştir)
        self.target1 = 1000
        self.target2 = 1000

        self.prev_error1 = 0
        self.prev_error2 = 0

        self.kp = 0.1
        self.kd = 0.05

        self.uart = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

        self.subscription = self.create_subscription(
            Int32MultiArray,
            'encoder_data',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        pos1 = msg.data[0]
        pos2 = msg.data[1]

        # Hatalar
        error1 = self.target1 - pos1
        error2 = self.target2 - pos2

        # PD kontrol hesaplama
        control_signal1 = self.kp * error1 + self.kd * (error1 - self.prev_error1)
        control_signal2 = self.kp * error2 + self.kd * (error2 - self.prev_error2)

        self.prev_error1 = error1
        self.prev_error2 = error2

        # Saturasyon
        control_signal1 = max(-100, min(100, int(control_signal1)))
        control_signal2 = max(-100, min(100, int(control_signal2)))

        # UART ile Pico'ya gönder
        command = f"{control_signal1},{control_signal2}\n"
        self.uart.write(command.encode())
        self.get_logger().info(f"Sent: {command.strip()}")

def main(args=None):
    rclpy.init(args=args)
    node = PDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
