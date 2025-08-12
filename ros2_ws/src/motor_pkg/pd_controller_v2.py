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
        self.kp = self.declare_parameter('kp', 0.3).get_parameter_value().double_value
        self.kd = self.declare_parameter('kd', 0.1).get_parameter_value().double_value

        self.prev_error1 = 0.0
        self.prev_error2 = 0.0

        self.dead = 20.0     # altı: 0 ver
        self.min_run = 40.0  # dead < |u| < min_run: min_run uygula

        self.sub_enc = self.create_subscription(Int32MultiArray, 'encoder_data', self.enc_cb, 10)
        self.pub_cmd = self.create_publisher(Int32MultiArray, 'motor_cmd', 10)

        self.get_logger().info(f"PD ready: kp={self.kp}, kd={self.kd}, targets=({self.target1},{self.target2})")

    def _sat(self, u: float, low_end=-100.0, high_end=100.0) -> float:
        return high_end if u > high_end else low_end if u < low_end else u

    def _shape(self, u: float) -> int:
        # |u| < dead -> 0
        # dead <= |u| < min_run -> sign(u)*min_run
        # else -> u (saturate)
        s = 1 if u >= 0 else -1
        a = abs(u)
        if a < self.dead:
            return 0
        if a < self.min_run:
            return int(s * self.min_run)
        return int(self._sat(u, -100.0, 100.0))

    def enc_cb(self, msg: Int32MultiArray):
        pos1, pos2 = msg.data[0], msg.data[1]

        err1 = self.target1 - pos1
        err2 = self.target2 - pos2

        u1 = self.kp * err1 + self.kd * (err1 - self.prev_error1)
        u2 = self.kp * err2 + self.kd * (err2 - self.prev_error2)

        self.prev_error1 = err1
        self.prev_error2 = err2

        # Saturation ve int'e çevir
        u1 = self._shape(u1)
        u2 = self._shape(u2)

        self.get_logger().info(f"u1={u1}, u2={u2}  (err1={err1}, err2={err2})")

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
