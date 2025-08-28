#!/usr/bin/env python3
import rclpy, time
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Joy

class F710Teleop(Node):
    def __init__(self):
        super().__init__('f710_to_motor_cmd')

        # ---- Parametreler ----
        self.axis_y     = self.declare_parameter('axis_y', 1).value     # sol çubuk Y
        self.btn_lb     = self.declare_parameter('btn_lb', 4).value     # LB = geri sabit
        self.btn_rb     = self.declare_parameter('btn_rb', 5).value     # RB = ileri sabit
        self.btn_stop   = self.declare_parameter('btn_stop', 0).value   # A = stop
        self.invert_y   = bool(self.declare_parameter('invert_y', True).value)  # çoğu gamepad'de yukarı = -1
        self.max_percent= int(self.declare_parameter('max_percent', 60).value)  # 0..100
        self.deadzone   = float(self.declare_parameter('deadzone', 0.15).value)
        self.ramp_per_s = float(self.declare_parameter('ramp_per_s', 150.0).value)  # %/s hız sınırı
        self.watchdog_s = float(self.declare_parameter('watchdog_s', 0.5).value)

        # ---- ROS I/O ----
        self.pub = self.create_publisher(Int32MultiArray, 'motor_cmd', 10)
        self.sub = self.create_subscription(Joy, 'joy', self.on_joy, 10)

        self._last_cmd = 0.0
        self._last_pub_t = time.monotonic()
        self.create_timer(0.05, self._tick)  # 20 Hz watchdog

        self.get_logger().info("F710 teleop hazır (sol çubuk Y, LB/RB, A-stop)")

    def _shape(self, x: float) -> float:
        """deadzone+kübik şekil: hassas orta bölge."""
        if abs(x) < self.deadzone:
            return 0.0
        # yeniden ölçekle
        s = (abs(x) - self.deadzone) / (1.0 - self.deadzone)
        s = s**3  # ince kontrol
        return s if x > 0 else -s

    def _ramp(self, target: float) -> float:
        now = time.monotonic()
        dt = now - self._last_pub_t
        self._last_pub_t = now
        max_delta = self.ramp_per_s * dt
        cur = self._last_cmd
        delta = target - cur
        if delta >  max_delta: delta =  max_delta
        if delta < -max_delta: delta = -max_delta
        cur += delta
        self._last_cmd = cur
        return cur

    def on_joy(self, msg: Joy):
        # Öncelik: acil dur -> sabit LB/RB -> analog çubuk
        u = 0.0
        if self._btn(msg, self.btn_stop):
            u = 0.0
        elif self._btn(msg, self.btn_rb):
            u = +1.0
        elif self._btn(msg, self.btn_lb):
            u = -1.0
        else:
            val = self._axis(msg, self.axis_y)
            if self.invert_y:
                val = -val
            u = self._shape(val)

        target = int(round(self.max_percent * u))
        cmd = int(round(self._ramp(target)))

        self._publish_four(cmd)

    def _tick(self):
        # Watchdog: joy gelmediyse stop
        if time.monotonic() - self._last_pub_t > self.watchdog_s:
            self._last_cmd = 0.0
            self._publish_four(0)

    def _publish_four(self, value: int):
        msg = Int32MultiArray()
        msg.data = [value, value, value, value]
        self.pub.publish(msg)

    @staticmethod
    def _axis(msg: Joy, idx: int) -> float:
        try:   return float(msg.axes[idx])
        except: return 0.0

    @staticmethod
    def _btn(msg: Joy, idx: int) -> bool:
        try:   return (msg.buttons[idx] == 1)
        except: return False

def main():
    rclpy.init()
    node = F710Teleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
