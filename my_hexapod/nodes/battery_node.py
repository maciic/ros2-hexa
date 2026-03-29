import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

try:
    from smbus2 import SMBus

    IMPORT_SUCCESS = True
except ImportError:
    IMPORT_SUCCESS = False


class BatteryNode(Node):
    def __init__(self):
        super().__init__("battery_node")

        self.voltage_pub = self.create_publisher(Float32, "battery_voltage", 10)
        self.pct_pub = self.create_publisher(
            Float32, "battery_percent", 10
        )  # ÚJ: Foxglove Gauge-hoz
        self.status_pub = self.create_publisher(String, "battery_status", 10)
        self.cmd_pub = self.create_publisher(
            String, "robot_command", 10
        )  # ÚJ: Parancsok az Agynak

        self.timer = self.create_timer(2.0, self.timer_callback)

        self.hardware_available = IMPORT_SUCCESS
        self.I2C_BUS = 1
        self.INA3221_ADDR = 0x42
        self.CH1_BUS_V_REG = 0x02

        self.dead_triggered = False
        self.low_volt_ticks = (
            0  # Számláló, hogy ne egy pillanatnyi tüske miatt álljon le
        )

        if self.hardware_available:
            try:
                self.bus = SMBus(self.I2C_BUS)
                # Teszt olvasás: Ha itt elszáll, nincs rákötve az I2C kábel
                self.bus.read_word_data(self.INA3221_ADDR, self.CH1_BUS_V_REG)
                self.get_logger().info("INA3221 Akkumulátor Monitor Inicializálva 🔋")
            except Exception as e:
                self.get_logger().warn(
                    "⚠️ Nincs INA3221 a buszon! DUMMY MÓD aktiválva (12.0V)."
                )
                self.hardware_available = False
        else:
            self.get_logger().warn("⚠️ smbus2 hiányzik! DUMMY MÓD aktiválva.")

    def timer_callback(self):
        voltage = 12.0  # Dummy feszültség alapértelmezésben

        if self.hardware_available:
            try:
                word = self.bus.read_word_data(self.INA3221_ADDR, self.CH1_BUS_V_REG)
                word = ((word & 0xFF) << 8) | (word >> 8)
                voltage = (word >> 3) * 0.008
            except Exception as e:
                self.get_logger().error(f"Olvasási hiba az INA3221-ről: {e}")
                return

        # Töltöttség számítása (3S LiPo)
        MAX_V = 12.6
        MIN_V = 10.5
        pct = max(0.0, min(100.0, ((voltage - MIN_V) / (MAX_V - MIN_V)) * 100.0))

        # 1. Adatok publikálása
        self.voltage_pub.publish(Float32(data=voltage))
        self.pct_pub.publish(Float32(data=pct))  # Ezt húzd be a Foxglove Gauge-ba!

        # 2. Szöveges státusz és LOGOLÁS
        s_msg = String()
        if pct < 15: 
            s_msg.data = f"VESZÉLY! Akku: {voltage:.2f}V ({int(pct)}%)"
            if self.hardware_available: 
                self.get_logger().error(s_msg.data)
        elif pct < 30: 
            s_msg.data = f"FIGYELMEZTETÉS: Akku: {voltage:.2f}V ({int(pct)}%)"
            if self.hardware_available: 
                self.get_logger().warn(s_msg.data)
        else: 
            s_msg.data = f"Akku OK: {voltage:.2f}V ({int(pct)}%)"
            # Ide be is tehetünk egy kis trükköt, hogy ne spammelje tele a konzolt 2 másodpercenként:
            # Csak akkor írja ki az INFO-t, ha hardveres, és te akarod (de hagyhatjuk simán is)
            if self.hardware_available: 
                self.get_logger().info(s_msg.data)
                
        self.status_pub.publish(s_msg)

        # 3. VÉSZLEÁLLÍTÓ LOGIKA (Csak élő hardvernél)
        if self.hardware_available and voltage <= 10.5:
            self.low_volt_ticks += 1
            if (
                self.low_volt_ticks >= 3 and not self.dead_triggered
            ):  # 3x (6 másodpercig) kell mélyponton lennie
                self.dead_triggered = True
                self.cmd_pub.publish(String(data="SYS_DEAD"))
                self.get_logger().error(
                    "☠️ AKKU KRITIKUS (10.5V)! Leállási parancs kiküldve az Agynak!"
                )
        elif self.hardware_available and voltage > 10.6:
            self.low_volt_ticks = 0


def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()