import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import json
import os
import math
import time

# Hardveres importok (a test4.py alapján)
try:
    from board import SCL, SDA
    import busio
    from adafruit_pca9685 import PCA9685
    from adafruit_motor import servo
    IMPORT_SUCCESS = True
except ImportError:
    IMPORT_SUCCESS = False

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

        # 2. HARDVER INICIALIZÁLÁS (test4.py alapján)
        if self.hardware_available:
            try:
                self.i2c = busio.I2C(SCL, SDA)
                self.pca = PCA9685(self.i2c)
                self.pca.frequency = self.config.get('frequency', 50)
                
                # Szervó objektumok létrehozása a map alapján
                for name, cfg in self.servo_map.items():
                    pin = cfg['pin']
                    # Csak akkor hozzuk létre, ha még nincs és létező csatorna (0-15)
                    if pin < 16: 
                        # Létrehozzuk az adafruit_motor szervó objektumot
                        self.servos[name] = servo.Servo(self.pca.channels[pin])
                        # Opcionális: Min/Max pulse width beállítás, ha kell
                        # self.servos[name].set_pulse_width_range(min_pulse=500, max_pulse=2500)
                
                self.get_logger().info(f"PCA9685 Inicializálva ({len(self.servos)} szervóval) 🔌")
                
            except Exception as e:
                self.get_logger().error(f"HARDVER HIBA: {e}")
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
        deg = math.degrees(angle_rad)
        
        # 1. Hozzáadjuk a fizikai offsetet (pl. a Femur +20 foka)
        deg_offset = deg + cfg['offset']
        
        # 2. Irány és Center alkalmazása
        # Ha dir = -1, akkor: center - deg_offset
        # Ha dir =  1, akkor: center + deg_offset
        final_angle = cfg['center'] + (deg_offset * cfg['dir'])
        
        return max(0.0, min(180.0, final_angle))

    def listener_callback(self, msg):
        # msg.name: ['joint_1_coxa', ...]
        # msg.position: [0.5, ...] (radiánban)
        
        for i, name in enumerate(msg.name):
            if name in self.servos and self.hardware_available:
                angle_rad = msg.position[i]
                cfg = self.servo_map[name]
                
                # Konverzió
                target_angle = self.rad_to_degree_mapped(angle_rad, cfg)
                
                # Küldés a motornak
                try:
                    self.servos[name].angle = target_angle
                    self.get_logger().info(f"Servo {name} angle: {target_angle}")
                except ValueError:
                    # Ha a szög kívül esik a tartományon, a library hibát dobhat
                    pass
            elif name in self.servo_map and not self.hardware_available:
                # Dummy log csak az 1-es lábra, hogy ne floodoljon
                if "joint_1" in name:
                     cfg = self.servo_map[name]
                     deg = self.rad_to_degree_mapped(msg.position[i], cfg)
                     self.get_logger().info(f"{name}: {deg:.1f}°")

    def __del__(self):
        # Destruktor: Ha leállítjuk a node-ot, engedje el a motorokat
        if self.hardware_available and hasattr(self, 'pca'):
            self.get_logger().info("Szervók elengedése...")
            self.pca.deinit()

def main(args=None):
    rclpy.init(args=args)
    node = ServoNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()