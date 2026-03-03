import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import json
import os
import math
import time
import traceback

# Rántsuk le a leplet a rejtett import hibáról!
try:
    from board import SCL, SDA
    import busio
    from adafruit_pca9685 import PCA9685
    from adafruit_motor import servo
    IMPORT_SUCCESS = True
    print("✅ Adafruit hardver konyvtarak sikeresen betoltve!")
except Exception as e:
    IMPORT_SUCCESS = False
    print("❌ IMPORT HIBA TÖRTÉNT:")
    print(f"Hiba oka: {e}")
    print(traceback.format_exc())
    print("--------------------------------------------------")

class ServoNode(Node):
    def __init__(self):
        super().__init__('hexapod_hardware')
        
        self.hardware_available = IMPORT_SUCCESS

        # 1. KONFIG BETÖLTÉSE
        script_dir = os.path.dirname(os.path.realpath(__file__))
        config_path = os.path.join(script_dir, '..', 'config', 'servo_map.json')
        
        with open(config_path, 'r') as f:
            self.config = json.load(f)
        
        self.servo_map = self.config['servos']
        self.servos = {} # Itt tároljuk a szervó objektumokat
        self.pcas = {}   # Itt tároljuk a PCA9685 paneleket az I2C címük alapján

        # 2. HARDVER INICIALIZÁLÁS (test4.py alapján)
        if self.hardware_available:
            try:
                self.i2c = busio.I2C(SCL, SDA)
                freq = self.config.get('frequency', 50)
                
                self.get_logger().info("I2C busz megnyitva, szervóvezérlők keresése...")

                # --- KIFEJEZETT PÉLDÁNYOSÍTÁS MINDKÉT PANELRE ---
                
                # 1. Panel (Bal oldal szervói - 0x40 / 64)
                try:
                    pca1 = PCA9685(self.i2c, address=0x40)
                    pca1.frequency = freq
                    self.pcas[64] = pca1
                    self.get_logger().info("1. PCA9685 (0x40) SIKERESEN inicializálva! 🟢")
                except Exception as e:
                    self.get_logger().error(f"Hiba az 1. panel (0x40) elérésekor: {e}")

                # 2. Panel (Jobb oldal szervói - 0x41 / 65)
                try:
                    pca2 = PCA9685(self.i2c, address=0x41)
                    pca2.frequency = freq
                    self.pcas[65] = pca2
                    self.get_logger().info("2. PCA9685 (0x41) SIKERESEN inicializálva! 🟢")
                except Exception as e:
                    self.get_logger().error(f"Hiba a 2. panel (0x41) elérésekor: {e}")

                # Ha egyik panel sincs meg, álljunk le
                if not self.pcas:
                    raise RuntimeError("Egyetlen PCA9685 panelt sem sikerult elinditani a buszon!")

                # --- SZERVÓK RÁKÖTÉSE A PANELEKRE ---
                for name, cfg in self.servo_map.items():
                    board_addr = cfg.get('board', 64) 
                    pin = cfg['pin']
                    
                    # Csak akkor hozzuk létre, ha a panel él, és létező csatorna (0-15)
                    if board_addr in self.pcas and pin < 16: 
                        # Létrehozzuk az adafruit_motor szervó objektumot a megfelelő panelen
                        self.servos[name] = servo.Servo(self.pcas[board_addr].channels[pin])
                
                self.get_logger().info(f"Összesen {len(self.servos)} szervó inicializálva! 🔌")
                
            except Exception as e:
                self.get_logger().error(f"KIRITIKUS HARDVER HIBA: {e}")
                self.get_logger().error(traceback.format_exc()) # <--- Ez kiírja a pontos hiba okát!
                self.hardware_available = False
        
        if not self.hardware_available:
            self.get_logger().warn("⚠️ Nincs I2C hardver (DUMMY MÓD) - Csak szimuláció")

        # 3. ROS FELIRATKOZÁS
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10
        )

    def rad_to_degree_mapped(self, angle_rad, cfg):
        """
        Átszámolja a ROS radiánt a test4.py szerinti fokokra.
        Logika: SzervóSzög = Center + (Fok + Offset) * Irány
        Példa a test4.py alapján: 180 - (fok + 20)
        Itt: Center=180, Irány=-1, Offset=20
        """
        deg = round(math.degrees(angle_rad))
        deg_offset = deg + cfg['offset']
        final_angle = cfg['center'] + (deg_offset * cfg['dir'])
        return max(0.0, min(180.0, final_angle))

    def listener_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.servos and self.hardware_available:
                angle_rad = msg.position[i]
                cfg = self.servo_map[name]
                
                target_angle = self.rad_to_degree_mapped(angle_rad, cfg)
                
                # --- DEBUG LOG ---
                if "joint_1" in name and "coxa" in name:
                     pass
                     #self.get_logger().info(f"[{name}] Rad: {angle_rad:.3f} -> Deg: {target_angle:.1f}")
                
                try:
                    self.servos[name].angle = target_angle
                except ValueError:
                    pass
            elif name in self.servo_map and not self.hardware_available:
                if "joint_1" in name:
                     cfg = self.servo_map[name]
                     deg = self.rad_to_degree_mapped(msg.position[i], cfg)
                     #self.get_logger().info(f"{name}: {deg:.1f}°")

    def shutdown_servos(self):
        """Kifejezetten leállítja a PWM jeleket és elengedi a motorokat"""
        if self.hardware_available:
            self.get_logger().info("🛑 Program leállítása: Szervók elengedése...")
            
            # 1. Minden motor PWM jelének nullázása (ettől azonnal elernyednek)
            for name, s in self.servos.items():
                try:
                    s.angle = None
                except:
                    pass
            
            # 2. Az I2C chipek (PCA9685) lekapcsolása
            for addr, pca in self.pcas.items():
                try:
                    pca.deinit()
                except:
                    pass

def main(args=None):
    rclpy.init(args=args)
    node = ServoNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Amikor nyomsz egy Ctrl+C-t a terminálban, ide ugrik a vezérlés
        node.get_logger().info("Ctrl+C érzékelve, leállási folyamat indítása...")
    finally:
        # A finally blokk GARANTÁLTAN lefut a kilépés előtt!
        node.shutdown_servos() 
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()