import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import time
import math

try:
    from smbus2 import SMBus
    IMPORT_SUCCESS = True
except ImportError:
    IMPORT_SUCCESS = False

class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        
        self.publisher_ = self.create_publisher(Imu, 'imu/data_raw', 10)
        
        # 50 Hz-es frissítés (0.02 másodperc) - Gyorsabb is lehet, de a teszthez ez tökéletes
        self.timer = self.create_timer(0.02, self.timer_callback)

        self.hardware_available = IMPORT_SUCCESS
        self.I2C_BUS = 1
        self.MPU_ADDR = 0x68

        if self.hardware_available:
            try:
                self.bus = SMBus(self.I2C_BUS)
                
                # --- MPU6050 FELÉBRESZTÉSE ---
                # A 0x6B (PWR_MGMT_1) regiszterbe 0-t írunk, hogy kikapcsoljuk az alvó módot
                self.bus.write_byte_data(self.MPU_ADDR, 0x6B, 0x00)
                time.sleep(0.1)
                
                self.get_logger().info("✅ GY-521 (MPU6050) IMU sikeresen felébresztve a 0x68 címen!")
            except Exception as e:
                self.get_logger().error(f"❌ I2C Inicializálási hiba az IMU-nál: {e}")
                self.hardware_available = False
        else:
            self.get_logger().warn("⚠️ smbus2 hiányzik! (Dummy mód)")

    def read_raw_data(self, addr):
        """ Beolvas 2 byte-ot a megadott regiszterből és átalakítja előjeles (signed) 16-bites számmá """
        high = self.bus.read_byte_data(self.MPU_ADDR, addr)
        low = self.bus.read_byte_data(self.MPU_ADDR, addr+1)
        
        # Bitművelet: Összefűzzük a magas és alacsony byte-okat
        value = ((high << 8) | low)
        
        # Ha a szám negatív (2-es komplemens a 16-bites rendszerben)
        if value > 32768:
            value = value - 65536
        return value

    def timer_callback(self):
        if not self.hardware_available:
            return

        try:
            # 1. NYERS ADATOK KIOLVASÁSA A REGISZTEREKBŐL
            acc_x = self.read_raw_data(0x3B)
            acc_y = self.read_raw_data(0x3D)
            acc_z = self.read_raw_data(0x3F)
            
            gyro_x = self.read_raw_data(0x43)
            gyro_y = self.read_raw_data(0x45)
            gyro_z = self.read_raw_data(0x47)

            # 2. KONVERZIÓ FIZIKAI MÉRTÉKEGYSÉGEKRE
            # Gyorsulás: alapértelmezett ±2g méréshatárnál a szorzó 16384. 
            # Ezt be kell szorozni a gravitációs állandóval (9.80665), hogy m/s^2 legyen.
            GRAVITY = 9.80665
            ax = (acc_x / 16384.0) * GRAVITY
            ay = (acc_y / 16384.0) * GRAVITY
            az = (acc_z / 16384.0) * GRAVITY

            # Giroszkóp: alapértelmezett ±250 fok/mp méréshatárnál a szorzó 131.0.
            # Ezt radián/másodpercre (rad/s) kell váltani a ROS2 szabványhoz.
            gx = math.radians(gyro_x / 131.0)
            gy = math.radians(gyro_y / 131.0)
            gz = math.radians(gyro_z / 131.0)

            # 3. ROS2 ÜZENET ÖSSZEÁLLÍTÁSA
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "imu_link"

            msg.linear_acceleration.x = ax
            msg.linear_acceleration.y = ay
            msg.linear_acceleration.z = az

            msg.angular_velocity.x = gx
            msg.angular_velocity.y = gy
            msg.angular_velocity.z = gz

            # Mivel ez "raw" node, a dőlésszöget (orientation) nem mi számoljuk ki, 
            # azt majd a Kalman-szűrő csinálja. Ezért a kovariancia első elemét -1-re tesszük, 
            # jelezve a ROS-nak, hogy hagyja figyelmen kívül.
            msg.orientation_covariance[0] = -1.0 
            
            # A gyorsulásmérő varianciája (Mennyire bízunk benne? Nagyobb szám = jobban kiszűri a zajt, de lassabb)
            msg.linear_acceleration_covariance[0] = 0.04  # X
            msg.linear_acceleration_covariance[4] = 0.04  # Y
            msg.linear_acceleration_covariance[8] = 0.04  # Z

            # A giroszkóp varianciája (Általában sokkal pontosabb, kisebb zajjal)
            msg.angular_velocity_covariance[0] = 0.002 # X
            msg.angular_velocity_covariance[4] = 0.002 # Y
            msg.angular_velocity_covariance[8] = 0.002 # Z

            self.publisher_.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Olvasási hiba az IMU-ról: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()