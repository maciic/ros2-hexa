import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import time

try:
    from smbus2 import SMBus
    IMPORT_SUCCESS = True
except ImportError:
    IMPORT_SUCCESS = False

class BatteryNode(Node):
    def __init__(self):
        super().__init__('battery_node')
        
        # ROS Publisherek: Két külön csatornán küldjük az adatokat
        self.voltage_pub = self.create_publisher(Float32, 'battery_voltage', 10)
        self.status_pub = self.create_publisher(String, 'battery_status', 10)
        
        # 2 másodpercenként frissítünk
        self.timer = self.create_timer(2.0, self.timer_callback)

        self.hardware_available = IMPORT_SUCCESS
        self.I2C_BUS = 1
        self.INA3221_ADDR = 0x42  # Az átforrasztott címed
        self.CH1_BUS_V_REG = 0x02

        if self.hardware_available:
            try:
                self.bus = SMBus(self.I2C_BUS)
                self.get_logger().info("INA3221 Akkumulátor Monitor Inicializálva 🔋")
            except Exception as e:
                self.get_logger().error(f"I2C Inicializálási hiba: {e}")
                self.hardware_available = False
        else:
            self.get_logger().warn("⚠️ smbus2 hiányzik! Futtasd: pip install smbus2")

    def timer_callback(self):
        if not self.hardware_available:
            return

        try:
            # I2C Olvasás
            word = self.bus.read_word_data(self.INA3221_ADDR, self.CH1_BUS_V_REG)
            
            # Big-Endian -> Little-Endian konverzió és eltolás
            word = ((word & 0xFF) << 8) | (word >> 8)
            voltage = (word >> 3) * 0.008

            # Töltöttség számítása (3S LiPo)
            MAX_V = 12.6
            MIN_V = 9.9
            pct = max(0, min(100, int(((voltage - MIN_V) / (MAX_V - MIN_V)) * 100)))

            # 1. Nyers feszültség küldése (Float) - Jó lehet pl. Foxglove grafikonhoz
            v_msg = Float32()
            v_msg.data = voltage
            self.voltage_pub.publish(v_msg)

            # 2. Szöveges státusz küldése és logolás
            s_msg = String()
            if pct < 15:
                s_msg.data = f"VESZÉLY! Akku: {voltage:.2f}V ({pct}%)"
                self.get_logger().error(s_msg.data)  # Piros log a terminálban!
            elif pct < 30:
                s_msg.data = f"FIGYELMEZTETÉS: Akku merül: {voltage:.2f}V ({pct}%)"
                self.get_logger().warn(s_msg.data)   # Sárga log
            else:
                s_msg.data = f"Akku OK: {voltage:.2f}V ({pct}%)"
                # Ezt kikommentezheted, ha nem akarod, hogy 2 másodpercenként teleírja a terminált
                self.get_logger().info(s_msg.data)   
                
            self.status_pub.publish(s_msg)

        except Exception as e:
            self.get_logger().error(f"Olvasási hiba az INA3221-ről: {e}")

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

if __name__ == '__main__':
    main()