import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import serial

def wrap_err(a, b, M):
    # a-b farkını dairesel eksende [-M/2, M/2) aralığına getir (kısa yoldan fark)
    e = (a - b + M//2) % M - M//2
    return e

class PDController(Node):
    def __init__(self):
        super().__init__('pd_controller_phase_lock')

        # === ÖNEMLİ AYARLAR ===
        self.CPR = 2000              # 1 turdaki sayım (KENDİ ENKODERİNE GÖRE DOLDUR!)
        self.phase_deg = 30          # Motor2 = Motor1 - 30°
        self.offset_counts = round(self.CPR * self.phase_deg / 360.0)

        # Motor1 (lider) için örnek hedef (mutlak konum)
        self.target1 = 1000

        # PD kazançları (ufaktan başla, deneyerek ayarla)
        self.kp_abs = 0.10           # motor1 (mutlak hata)
        self.kd_abs = 0.05
        self.kp_phase = 0.15         # motor2 (faz hatası)
        self.kd_phase = 0.06

        # Geçmiş hata ve zaman
        self.prev_err1 = 0
        self.prev_err_phase = 0
        self.prev_t = None

        # Pico ile UART
        self.uart = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

        # Encoder verisini dinle
        self.subscription = self.create_subscription(
            Int32MultiArray, 'encoder_data', self.listener_cb, 10
        )

    def listener_cb(self, msg):
        # 1) Pozisyonları al (tam sayı)
        pos1 = int(msg.data[0])
        pos2 = int(msg.data[1])

        # 2) dt hesapla (türev için)
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.prev_t is None:
            self.prev_t = now
            return  # ilk gelişte kontrol üretmiyoruz
        dt = now - self.prev_t
        if dt <= 0:
            dt = 1e-3
        self.prev_t = now

        # 3) Motor1 = lider: klasik PD (mutlak hedefe doğru)
        err1 = self.target1 - pos1
        derr1 = (err1 - self.prev_err1) / dt
        u1 = self.kp_abs * err1 + self.kd_abs * derr1
        self.prev_err1 = err1

        # 4) Motor2 = takipçi: "faz farkı" 30° geride tut
        ang1 = pos1 % self.CPR
        ang2 = pos2 % self.CPR
        ref2 = (ang1 - self.offset_counts) % self.CPR

        err_phase = wrap_err(ang2, ref2, self.CPR)
        derr_phase = (err_phase - self.prev_err_phase) / dt
        u2 = self.kp_phase * err_phase + self.kd_phase * derr_phase
        self.prev_err_phase = err_phase

        # 5) Sinyalleri sınırla ve tamsayı yap
        def sat(x): return max(-100, min(100, int(x)))
        u1 = sat(u1)
        u2 = sat(u2)

        # 6) UART ile Pico'ya gönder
        cmd = f"{u1},{u2}\n"
        try:
            self.uart.write(cmd.encode())
        except Exception as e:
            self.get_logger().error(f"UART write failed: {e}")

        # 7) (İsteğe bağlı) debug log
        self.get_logger().info(
            f"u1={u1} u2={u2} | ang1={ang1} ang2={ang2} ref2={ref2} ephi={err_phase}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = PDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
