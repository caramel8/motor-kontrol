#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

def wrap_err(a, b, M):
    # a-b farkını dairesel eksende [-M/2, M/2)
    return (a - b + M//2) % M - M//2

class PhaseLockPD(Node):
    def __init__(self):
        super().__init__('pd_controller_phase_lock')

        # === Parametreler ===
        self.cpr        = self.declare_parameter('cpr', 2000).get_parameter_value().integer_value
        self.phase_deg  = self.declare_parameter('phase_deg', 30.0).get_parameter_value().double_value
        self.target1    = self.declare_parameter('target1', 1000).get_parameter_value().integer_value

        self.kp_abs     = self.declare_parameter('kp_abs', 0.10).get_parameter_value().double_value
        self.kd_abs     = self.declare_parameter('kd_abs', 0.05).get_parameter_value().double_value
        self.kp_phase   = self.declare_parameter('kp_phase', 0.15).get_parameter_value().double_value
        self.kd_phase   = self.declare_parameter('kd_phase', 0.06).get_parameter_value().double_value

        # Senin net kuralın: |u|<20 -> 0 ; 20<=|u|<40 -> sign*40 ; >=40 -> u
        self.dead       = self.declare_parameter('dead', 20.0).get_parameter_value().double_value
        self.min_run    = self.declare_parameter('min_run', 40.0).get_parameter_value().double_value
        self.log        = self.declare_parameter('log', True).get_parameter_value().bool_value

        # Faz offset (counts)
        self.offset_counts = round(self.cpr * self.phase_deg / 360.0)

        # Durum
        self.prev_err1 = 0.0
        self.prev_err_phase = 0.0
        self.prev_t = None

        # ROS I/O
        self.sub_enc = self.create_subscription(Int32MultiArray, 'encoder_data', self.enc_cb, 10)
        self.pub_cmd = self.create_publisher(Int32MultiArray, 'motor_cmd', 10)

        self.get_logger().info(
            f"Phase-Lock PD ready | cpr={self.cpr}, phase_deg={self.phase_deg} "
            f"(offset={self.offset_counts}), target1={self.target1} | "
            f"kp_abs={self.kp_abs}, kd_abs={self.kd_abs}, kp_phase={self.kp_phase}, kd_phase={self.kd_phase} | "
            f"dead={self.dead}, min_run={self.min_run}"
        )

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
        # 1) Pozisyonları al
        pos1 = int(msg.data[0])
        pos2 = int(msg.data[1])

        # 2) dt
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.prev_t is None:
            self.prev_t = now
            return
        dt = now - self.prev_t
        if dt <= 0:
            dt = 1e-3
        self.prev_t = now

        # 3) Motor1: mutlak hedefe PD
        err1 = self.target1 - pos1
        derr1 = (err1 - self.prev_err1) / dt
        u1 = self.kp_abs * err1 + self.kd_abs * derr1
        self.prev_err1 = err1

        # 4) Motor2: faz farkını (ang2 ~ ang1 - offset) kilitle
        ang1 = pos1 % self.cpr
        ang2 = pos2 % self.cpr
        ref2 = (ang1 - self.offset_counts) % self.cpr

        err_phase = wrap_err(ang2, ref2, self.cpr)
        derr_phase = (err_phase - self.prev_err_phase) / dt
        u2 = self.kp_phase * err_phase + self.kd_phase * derr_phase
        self.prev_err_phase = err_phase

        # 5) Şekillendir ve yayınla
        u1 = self._shape_simple(u1)
        u2 = self._shape_simple(u2)

        out = Int32MultiArray()
        out.data = [u1, u2]
        self.pub_cmd.publish(out)

        if self.log:
            self.get_logger().info(
                f"u1={u1} u2={u2} | pos1={pos1} pos2={pos2} | ang1={ang1} ref2={ref2} ephi={err_phase}"
            )

def main(args=None):
    rclpy.init(args=args)
    node = PhaseLockPD()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
