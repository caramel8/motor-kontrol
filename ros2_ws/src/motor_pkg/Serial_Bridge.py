#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        # Parametreler
        port = self.declare_parameter('port', '/dev/ttyACM0').get_parameter_value().string_value
        baud = self.declare_parameter('baud', 115200).get_parameter_value().integer_value
        self.echo = self.declare_parameter('echo', True).get_parameter_value().bool_value  # ekrana yaz

        # Seri portu aç
        self.ser = serial.Serial(port, baudrate=baud, timeout=0.1, write_timeout=0.1)
        try:
            # MicroPython USB-CDC çoğu sistemde DTR=TRUE istiyor
            self.ser.dtr = True
            self.ser.rts = False
        except Exception:
            pass

        # Pub/Sub
        self.pub_enc = self.create_publisher(Int32MultiArray, 'encoder_data', 10)
        self.sub_cmd = self.create_subscription(Int32MultiArray, 'motor_cmd', self.cmd_cb, 10)

        # 100 Hz döngü
        self.timer = self.create_timer(0.01, self.loop)

        self.get_logger().info(f'Opened {port} @ {baud}')

    def loop(self):
        # 1) Satırı oku
        line = self.ser.readline()
        if not line:
            return

        s = line.decode('ascii', errors='ignore').strip()

        # 2) Parse etmeyi dene
        try:
            a_str, b_str = s.split(',', 1)
            a = int(a_str.strip())
            b = int(b_str.strip())
        except Exception:
            # parse edemediysek (ör. traceback/çöp satır), istersen göster
            if self.echo:
                self.get_logger().info(f'RAW: {s}')
            return

        # 3) Ekrana yaz + publish et
        if self.echo:
            self.get_logger().info(f'ENC: {a},{b}')

        msg = Int32MultiArray()
        msg.data = [a, b]
        self.pub_enc.publish(msg)

    def cmd_cb(self, msg: Int32MultiArray):
        # Gelen komutu pico'ya geri yaz (opsiyonel ama kalsın)
        try:
            m1 = int(max(-100, min(100, msg.data[0])))
            m2 = int(max(-100, min(100, msg.data[1])))
            self.ser.write(f"{m1},{m2}\r\n".encode('ascii', errors='ignore'))
            self.ser.flush()
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
