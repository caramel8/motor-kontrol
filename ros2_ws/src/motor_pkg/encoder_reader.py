import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial

class EncoderReader(Node):
    def __init__(self):
        super().__init__('encoder_reader')
        self.publisher_ = self.create_publisher(Int32, 'encoder_position', 10)
        
        # Raspberry Pi'deki Pico'nun bağlı olduğu port (kontrol et)
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

        self.timer = self.create_timer(0.05, self.read_encoder)  # 20 Hz

    def read_encoder(self):
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode().strip()
                position = int(line)
                msg = Int32()
                msg.data = position
                self.publisher_.publish(msg)
                self.get_logger().info(f'Published: {position}')
            except ValueError:
                pass  # Bozuk veri varsa atla

def main(args=None):
    rclpy.init(args=args)
    node = EncoderReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
