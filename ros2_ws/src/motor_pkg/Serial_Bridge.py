#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial
import time

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        # --- Serial: USB-CDC (tek sahip burası) ---
        port = self.declare_parameter('port', '/dev/ttyACM0').get_parameter_value().string_value
        baud = self.declare_parameter('baud', 115200).get_parameter_value().integer_value
        self.ser = serial.Serial(port, baudrate=baud, timeout=0.05)  # kısa timeout

        # --- Pub/Sub ---
        self.pub_enc = self.create_publisher(Int32MultiArray, 'encoder_data', 10)
        self.sub_cmd = self.create_subscription(Int32MultiArray, 'motor_cmd', self.cmd_cb, 10)

        # --- 100 Hz okuma timer'ı ---
        self.timer = self.create_timer(0.01, self.read_encoder)

        self.last_warn = 0.0  # hata spam koruması
        self.get_logger().info(f'Opened serial {port} @ {baud}')

    def read_encoder(self):
        try:
            if self.ser.in_waiting:
                line = self.ser.readline().decode(errors='ignore').strip()
                if not line:
                    return
                if ',' not in line:
                    # Başka bir debug çıktısı gelirse sessiz geç
                    return
                p1s, p2s = line.split(',', 1)
                msg = Int32MultiArray()
                msg.data = [int(p1s), int(p2s)]
                self.pub_enc.publish(msg)
        except Exception as e:
            # 1 sn’de bir uyar
            now = time.time()
            if now - self.last_warn > 1.0:
                self.get_logger().warn(f"Error parsing encoder data: {e}")
                self.last_warn = now

    def cmd_cb(self, msg: Int32MultiArray):
        try:
            # Beklenen: [m1, m2] aralığı -100..100
            m1 = int(max(-100, min(100, msg.data[0])))
            m2 = int(max(-100, min(100, msg.data[1])))
            cmd = f"{m1},{m2}\n".encode()
            self.ser.write(cmd)
        except Exception as e:
            self.get_logger().warn(f"Error writing command: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
