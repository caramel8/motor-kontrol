#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class PDController(Node):
    def __init__(self):
        super().__init__('pd_controller')

        # Targets
        self.target1 = self.declare_parameter('target1', 1000).get_parameter_value().integer_value
        self.target2 = self.declare_parameter('target2', 1000).get_parameter_value().integer_value

        # PD gains
        self.kp = self.declare_parameter('kp', 0.1).get_parameter_value().double_value
        self.kd = self.declare_parameter('kd', 0.05).get_parameter_value().double_value

        self.prev_error1 = 0.0
        self.prev_error2 = 0.0

        self.sub_enc = self.create_subscription(Int32MultiArray, 'encoder_data', self.enc_cb, 10)
        self.pub_cmd = self.create_publisher(Int32MultiArray, 'motor_cmd', 10)

        self.get_logger().info(f"PD ready: kp={self.kp}, kd={self.kd}, targets=({self.target1},{self.target2})")

    def enc_cb(self, msg: Int32MultiArray):
        pos1, pos2 = msg.data[0], msg.data[1]

        err1 = self.target1 - pos1
        err2 = self.target2 - pos2

        u1 = self.kp * err1 + self.kd * (err1 - self.prev_error1)
        u2 = self.kp * err2 + self.kd * (err2 - self.prev_error2)

        self.prev_error1 = err1
        self.prev_error2 = err2

        # Saturation ve int'e çevir
        u1 = int(max(-100, min(100, u1)))
        u2 = int(max(-100, min(100, u2)))

        out = Int32MultiArray()
        out.data = [u1, u2]
        self.pub_cmd.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = PDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
