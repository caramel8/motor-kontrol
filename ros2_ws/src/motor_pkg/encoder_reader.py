import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial

class EncoderReader(Node):
    def __init__(self):
        super().__init__('encoder_reader')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'encoder_data', 10)

        # Pico ile UART haberleşme (ttyACM0 ya da ttyUSB0 olabilir)
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

        self.timer = self.create_timer(0.01, self.read_encoder_data)

    def read_encoder_data(self):
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode().strip()
                if ',' in line:
                    pos1_str, pos2_str = line.split(',')
                    msg = Int32MultiArray()
                    msg.data = [int(pos1_str), int(pos2_str)]
                    self.publisher_.publish(msg)
            except Exception as e:
                self.get_logger().warn(f"Error parsing encoder data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = EncoderReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
