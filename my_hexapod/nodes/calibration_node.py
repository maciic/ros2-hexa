import rclpy
from rclpy.node import Node
import threading
import json
import board
import busio
import os
import time
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

class BareMetalCalibrator(Node):
    def __init__(self):
        super().__init__('bare_metal_calibrator')
        
        self.is_released = False
        self.servo_map = {}
        self.global_freq = 50
        self.global_min_pulse = 500
        self.global_max_pulse = 2500
        self.load_servo_map()
        
        i2c = busio.I2C(board.SCL, board.SDA)
        
        # PONT MINT A RÉGI KÓDODBAN: self.pcas
        self.pcas = {
            64: PCA9685(i2c, address=64),
            65: PCA9685(i2c, address=65)
        }
        self.pcas[64].frequency = self.global_freq
        self.pcas[65].frequency = self.global_freq
        
        # PONT MINT A RÉGI KÓDODBAN: self.servos (Csak a bekötött motorokat inicializáljuk!)
        self.servos = {}
        for joint_name, config in self.servo_map.items():
            board_id = config['board']
            pin = config['pin']
            self.servos[joint_name] = servo.Servo(self.pcas[board_id].channels[pin])
        
        self.update_group('coxa', 90.0)
        self.update_group('femur', 0.0)
        self.update_group('tibia', 90.0)

    def load_servo_map(self):
        script_dir = os.path.dirname(os.path.realpath(__file__))
        map_path = os.path.join(script_dir, '..', 'config', 'servo_map.json')
        try:
            with open(map_path, 'r') as f:
                data = json.load(f)
                self.global_freq = data.get('frequency', 50)
                self.global_min_pulse = data.get('min_pulse', 500)
                self.global_max_pulse = data.get('max_pulse', 2500)
                self.servo_map = data['servos']
        except Exception as e:
            self.get_logger().error(f"❌ Nem tudom beolvasni a JSON-t: {e}")

    def update_single_servo(self, joint_name, logical_angle):
        if joint_name not in self.servo_map:
            return
            
        config = self.servo_map[joint_name]
        shifted_angle = logical_angle - 90.0
        scale = config.get('scale', 1.0)
        physical_angle = config['center'] + config['offset'] + (shifted_angle * config['dir'] * scale)
        
        pulse_range = self.global_max_pulse - self.global_min_pulse
        target_pulse_us = self.global_min_pulse + (physical_angle / 180.0) * pulse_range
        target_pulse_us = max(400.0, min(2600.0, target_pulse_us))
        
        period_us = 1000000.0 / self.global_freq
        duty_cycle = int((target_pulse_us / period_us) * 65535)
        
        # MEGJÁTSSZUK A RENDSZERT: 
        # Használjuk az Adafruit objektumot, de kikerüljük a 0-180 fokos limitjét
        # azzal, hogy közvetlenül a mögöttes PWM rétegbe írjuk az adatot!
        try:
            self.servos[joint_name]._pwm.duty_cycle = duty_cycle
        except Exception:
            pass

    def update_group(self, group_name, logical_angle):
        for joint_name in self.servo_map.keys():
            if group_name in joint_name:
                self.update_single_servo(joint_name, logical_angle)

    def release_all_servos(self):
        if self.is_released:
            return
        self.is_released = True
        print("\n\n⏳ BIZTONSÁGI LEÁLLÍTÁS: Szervók elengedése és I2C lekapcsolása...")
        
        # TÖKÉLETES MÁSOLATA A TE RÉGI KÓDODNAK:
        # Csak azokat a pineket nullázzuk, amiken biztosan motor van (nincs "sörétes puska")
        for name, s in self.servos.items():
            try:
                s.angle = None
            except Exception as e:
                print(f"⚠️ Hiba a {name} szervo elengedésekor: {e}")
                
        # Adunk egy kis időt, hogy az I2C parancsok végigmenjenek a kábelen
        time.sleep(0.3)
        
        # I2C Chipek lekapcsolása
        for addr, pca in self.pcas.items():
            try:
                pca.deinit()
                print(f"✔️  {addr}-es PCA9685 chip sikeresen lekapcsolva.")
            except Exception as e:
                print(f"⚠️ Hiba a {addr}-es chip lekapcsolásakor: {e}")
                
        print("🛑 Minden hardver biztonságosan leállítva. A robot mozgatható.\n")

    def terminal_interface(self):
        print("\n" + "="*55)
        print("🚀 BARE-METAL HEXAPOD KALIBRÁTOR 🚀")
        print("-" * 55)
        print("PARANCSOK: coxa 90, joint_1_coxa 0")
        print("KILÉPÉS: q, quit, vagy Ctrl+C (Garantáltan elengedi a motorokat!)")
        print("="*55 + "\n")
        
        while rclpy.ok():
            try:
                cmd_raw = input("Parancs: ").strip().lower()
                
                if cmd_raw in ['q', 'quit', 'exit']:
                    break
                    
                cmd = cmd_raw.split()
                if len(cmd) != 2:
                    continue
                
                target = cmd[0]
                angle = float(cmd[1])
                
                if 0 <= angle <= 180:
                    if target in ['coxa', 'femur', 'tibia']:
                        self.update_group(target, angle)
                        print(f"✔️  {target} csoport beállítva: {angle} fokra.")
                    elif target in self.servo_map:
                        self.update_single_servo(target, angle)
                        print(f"✔️  {target} beállítva: {angle} fokra.")
            
            # Ez azonnal elkapja a Ctrl+C-t és szépen kiugrik a ciklusból
            except KeyboardInterrupt:
                print("\n[Ctrl+C parancs érzékelve]")
                break
            except Exception:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = BareMetalCalibrator()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.terminal_interface()
    finally:
        node.release_all_servos()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()