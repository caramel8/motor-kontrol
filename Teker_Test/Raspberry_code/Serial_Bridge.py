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
        self.echo = self.declare_parameter('echo', True).get_parameter_value().bool_value

        # Seri port
        self.ser = serial.Serial(port, baudrate=baud, timeout=0.02, write_timeout=0.1)
        try:
            # MicroPython USB-CDC çoğu sistemde DTR=TRUE ister
            self.ser.dtr = True
            self.ser.rts = False
            self.ser.reset_input_buffer()
        except Exception:
            pass

        # ROS pub/sub
        self.pub_enc = self.create_publisher(Int32MultiArray, 'encoder_data', 10)  # [pos1,pos2]
        self.sub_cmd = self.create_subscription(Int32MultiArray, 'motor_cmd', self.cmd_cb, 10)

        # 100 Hz döngü
        self.timer = self.create_timer(0.01, self.loop)

        self.get_logger().info(f'Opened {port} @ {baud}')

    def loop(self):
        line = self.ser.readline()
        if not line:
            return

        s = line.decode('ascii', errors='ignore').strip()
        if not s:
            return

        parts = s.split(',')
        if len(parts) < 2:
            if self.echo:
                self.get_logger().info(f'RAW<2: {s}')
            return

        try:
            pos1 = int(parts[0].strip())
            pos2 = int(parts[1].strip())
        except Exception:
            if self.echo:
                self.get_logger().info(f'RAW_ERR: {s}')
            return

        # İsteğe bağlı: üçüncü alan "current" ise yakala (logla veya ayrı topiğe publish edebilirsin)
        if len(parts) >= 3 and self.echo:
            curr_txt = parts[2].strip()
            # current float olabilir; robust log
            self.get_logger().info(f'ENC: {pos1},{pos2}  I={curr_txt}')

        msg = Int32MultiArray()
        msg.data = [pos1, pos2]
        self.pub_enc.publish(msg)

    def cmd_cb(self, msg: Int32MultiArray):
        """[u1,u2] (-100..100) → 'u1,u2\\r\\n'"""
        try:
            data = list(msg.data)
            # Eksikse 0 ile tamamla, fazlaysa kırp
            if len(data) < 2:
                data += [0] * (2 - len(data))
            elif len(data) > 2:
                data = data[:2]

            u1 = int(max(-100, min(100, data[0])))
            u2 = int(max(-100, min(100, data[1])))

            out = f"{u1},{u2}\r\n".encode('ascii', errors='ignore')
            self.ser.write(out)
            self.ser.flush()

            if self.echo:
                self.get_logger().info(f'CMD → Pico: {u1},{u2}')
        except Exception as e:
            if self.echo:
                self.get_logger().info(f'WRITE_ERR: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
