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

        # 2. HARDVER INICIALIZÁLÁS
        if self.hardware_available:
            try:
                self.i2c = busio.I2C(SCL, SDA)
                freq = self.config.get('frequency', 50)
                
                # Globális min és max pulse beolvasása (alapértelmezett az Adafruit 500/2500)
                self.global_min_pulse = self.config.get('min_pulse', 500)
                self.global_max_pulse = self.config.get('max_pulse', 2500)
                
                self.get_logger().info("I2C busz megnyitva, szervóvezérlők keresése...")

                # --- 1. Panel (Bal oldal szervói - 0x40 / 64) ---
                try:
                    pca1 = PCA9685(self.i2c, address=0x40)
                    pca1.frequency = freq
                    self.pcas[64] = pca1
                    self.get_logger().info("1. PCA9685 (0x40) SIKERESEN inicializálva! 🟢")
                except Exception as e:
                    self.get_logger().error(f"Hiba az 1. panel (0x40) elérésekor: {e}")

                # --- 2. Panel (Jobb oldal szervói - 0x41 / 65) ---
                try:
                    pca2 = PCA9685(self.i2c, address=0x41)
                    pca2.frequency = freq
                    self.pcas[65] = pca2
                    self.get_logger().info("2. PCA9685 (0x41) SIKERESEN inicializálva! 🟢")
                except Exception as e:
                    self.get_logger().error(f"Hiba a 2. panel (0x41) elérésekor: {e}")

                if not self.pcas:
                    raise RuntimeError("Egyetlen PCA9685 panelt sem sikerult elinditani a buszon!")

                # --- SZERVÓK RÁKÖTÉSE A PANELEKRE ---
                for name, cfg in self.servo_map.items():
                    board_addr = cfg.get('board', 64) 
                    pin = cfg['pin']
                    
                    if board_addr in self.pcas and pin < 16: 
                        # Továbbra is inicializáljuk a Servo objektumot, mert a leállításhoz a .angle = None kell
                        self.servos[name] = servo.Servo(
                            self.pcas[board_addr].channels[pin],
                            min_pulse=self.global_min_pulse,
                            max_pulse=self.global_max_pulse
                        )
                
                self.get_logger().info(f"Összesen {len(self.servos)} szervó inicializálva! 🔌")
                
            except Exception as e:
                self.get_logger().error(f"KIRITIKUS HARDVER HIBA: {e}")
                self.get_logger().error(traceback.format_exc())
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
        
        self.last_positions = {name: 0.0 for name in self.servo_map.keys()}

    def calculate_duty_cycle(self, angle_rad, cfg):
        """
        Kiszámolja a nyers PWM jelet a ROS radiánból, figyelembe véve a scale-t és az offsetet.
        """
        # 1. Radián átváltása eltérésre (fok). A URDF 0 radián a logikai alapállásunk.
        # Itt kivettem a round() kerekítést, hogy sokkal simább (tizedfok pontos) legyen a mozgás!
        deg_deviation = math.degrees(angle_rad)
        scale = cfg.get('scale', 1.0)
        
        # 2. A valós fizikai szög kiszámítása - HAJSZÁLPONTOSAN úgy, mint a kalibrátorban!
        # (Center + Offset) + (Eltérés * Irány * Szorzó)
        physical_angle = cfg['center'] + cfg['offset'] + (deg_deviation * cfg['dir'] * scale)
        
        # 3. Szög átváltása mikroszekundumra (µs)
        pulse_range = self.global_max_pulse - self.global_min_pulse
        target_pulse_us = self.global_min_pulse + (physical_angle / 180.0) * pulse_range
        
        # Biztonsági limitálás a nyers hardvernek (400µs és 2600µs között, hogy le ne égjen)
        target_pulse_us = max(400.0, min(2600.0, target_pulse_us))
        
        # 4. Mikroszekundum átváltása 16-bites (0-65535) Duty Cycle értékké
        freq = self.config.get('frequency', 50)
        period_us = 1000000.0 / freq
        duty_cycle = int((target_pulse_us / period_us) * 65535)
        
        return duty_cycle, physical_angle

    def listener_callback(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.servos and self.hardware_available:
                angle_rad = msg.position[i]
                cfg = self.servo_map[name]
                
                # Nyers PWM és a (csak debug miatt) kiszámolt fizikai szög lekérése
                duty_cycle, physical_angle = self.calculate_duty_cycle(angle_rad, cfg)
                
                # Elmentjük az aktuális pozíciót a leállító animációhoz!
                self.last_positions[name] = angle_rad
                
                try:
                    # Kikerüljük az Adafruit szigorú ellenőrzését, és egyenesen a PWM regiszterbe írunk!
                    self.servos[name]._pwm.duty_cycle = duty_cycle
                except Exception as e:
                    pass
                    
            elif name in self.servo_map and not self.hardware_available:
                if "joint_1" in name:
                     cfg = self.servo_map[name]
                     _, physical_angle = self.calculate_duty_cycle(msg.position[i], cfg)
                     # self.get_logger().info(f"[SIM] {name}: {physical_angle:.1f}°")

    def shutdown_servos(self):
        """Leállítja a programot egy finom leültető animációval (FOKOKBAN megadva!), majd lekapcsolja a chipeket."""
        if self.hardware_available:
            self.get_logger().info("🛑 Program leállítása: Leültető animáció indítása...")
            
            # ==========================================================
            # 1. CÉL POZÍCIÓK MEGADÁSA (0-180 FOKOS RENDSZERBEN)
            # Pont úgy, mint a kalibrátorban! A 90.0 az egyenes alapállás.
            # Ezeket a számokat írd át úgy, hogy a pókod tökéletesen hasra feküdjön!
            # ==========================================================
            sit_targets_deg = {
                'coxa': 90.0, 
                'femur': 0.0,  
                'tibia': 135.0    
            }
            
            # Átváltjuk a célokat ROS radián eltérésre a háttérben (90 a 0.0 radián)
            sit_targets_rad = {
                'coxa': math.radians(sit_targets_deg['coxa'] - 90.0),
                'femur': math.radians(sit_targets_deg['femur'] - 90.0),
                'tibia': math.radians(sit_targets_deg['tibia'] - 25.0)
            }
            
            steps = 50          # Hány lépésből álljon az animáció (képkocka)
            duration = 1.5      # Hány másodperc alatt ereszkedjen le
            delay = duration / steps
            
            # 2. Az animáció ciklusa
            for step in range(1, steps + 1):
                for name, cfg in self.servo_map.items():
                    if 'coxa' in name:
                        target_rad = sit_targets_rad['coxa']
                    elif 'femur' in name:
                        target_rad = sit_targets_rad['femur']
                    elif 'tibia' in name:
                        target_rad = sit_targets_rad['tibia']
                    else:
                        target_rad = 0.0
                        
                    start_rad = self.last_positions.get(name, 0.0)
                    
                    # Lágy átmenet (interpoláció) a jelenlegi póz és a hasalás között
                    current_rad = start_rad + (target_rad - start_rad) * (step / steps)
                    
                    # Itt történik a csoda: a függvény ráhúzza az offsetet, scale-t és centert a JSON-ből!
                    duty_cycle, _ = self.calculate_duty_cycle(current_rad, cfg)
                    try:
                        self.servos[name]._pwm.duty_cycle = duty_cycle
                    except Exception:
                        pass
                        
                time.sleep(delay)
                
            self.get_logger().info("✔️ Robot leültetve. I2C chipek lekapcsolása...")
            
            # 3. Az I2C chipek (PCA9685) lekapcsolása
            for addr, pca in self.pcas.items():
                try:
                    pca.deinit()
                except:
                    pass
            
            self.get_logger().info("🛑 Minden hardver biztonságosan leállítva. (A DS3225 megtartja a pozíciót)")

def main(args=None):
    rclpy.init(args=args)
    node = ServoNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("\nCtrl+C érzékelve, leállási folyamat indítása...")
    finally:
        node.shutdown_servos() 
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()