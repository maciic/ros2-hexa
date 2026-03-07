import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist 
import json
import time
import math
import os

# === IMPORTÁLJUK A MODULOKAT ===
from my_hexapod.core.robot_kinematics import HexapodKinematics 
from my_hexapod.core.robot_gait import HexapodGait 

class HexapodController(Node):
    def __init__(self):
        super().__init__('hexapod_controller')
        
        # 1. KONFIG BETÖLTÉSE
        script_dir = os.path.dirname(os.path.realpath(__file__))
        # Kilépünk a 'nodes' mappából (..), be a 'config'-ba
        CONFIG_PATH = os.path.join(script_dir, '..', 'config', 'robot_dimension.json')
        
        with open(CONFIG_PATH, 'r') as f:
            self.config = json.load(f)

        self.LEGS = self.config['legs']

        # 2. SEGÉDOSZTÁLYOK INICIALIZÁLÁSA
        self.kinematics = HexapodKinematics(self.config)
        self.gait = HexapodGait()

        # 3. ÁLLAPOTVÁLTOZÓK
        self.cmd_vel = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.current_vel = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.ramp_step = 0.05
        
        self.body_rpy = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.breathe_z = 0.0

        # 4. ROS KOMMUNIKÁCIÓ
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 1)
        self.marker_pub = self.create_publisher(Marker, 'target_marker', 1)
        self.debug_pub = self.create_publisher(Twist, 'current_vel', 1)
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        self.timer = self.create_timer(1.0 / 50.0, self.timer_callback)
        self.start_time = time.time()
        
        self.get_logger().info("Hexapod Controller: Modular & Ready! 🚀")

    def cmd_vel_callback(self, msg):
        self.cmd_vel['x'] = msg.linear.x
        self.cmd_vel['y'] = msg.linear.y
        self.cmd_vel['yaw'] = msg.angular.z

    def ramp_value(self, current, target, step):
        if current < target:
            return min(current + step, target)
        elif current > target:
            return max(current - step, target)
        return target

    def process_leg(self, leg_key, leg_cfg, t):
        """ Egy láb teljes feldolgozása a modulok segítségével """
        
        # A. JÁRÁS (Gait Module)
        phase = self.gait.get_tripod_phase(leg_key, t)
        
        # Lekérjük a járás offszeteket
        off_walk, off_strafe, off_turn, off_z = self.gait.calculate_step_offset(
            phase, self.current_vel['x'], self.current_vel['y'], self.current_vel['yaw']
        )

        # B. POZÍCIÓ SZÁMÍTÁS (Test koordinátarendszerben)
        angle = leg_cfg["mount_angle_rad"]
        dist = self.gait.params['base_dist']
        
        # Alap pozíció
        neutral_x = dist * math.cos(angle)
        neutral_y = dist * math.sin(angle)

        # Forgás (Tangenciális vektor számítása)
        dx_rot = (-neutral_y / dist) * off_turn
        dy_rot = (neutral_x / dist) * off_turn

        # Végső CÉLPONT (Test koordinátarendszerben)
        # Ez az a pont a térben, ahova a láb végét tenni akarjuk (Body IK nélkül)
        target_x = neutral_x + off_walk + dx_rot
        target_y = neutral_y + off_strafe + dy_rot
        
        # Sima, időalapú Z magasság a szinuszgörbéből és a lélegzésből
        target_z = self.gait.params['base_height'] + off_z + self.breathe_z

        # C. KINEMATIKA (Kinematics Module)
        # 1. Transzformáljuk a pontot a láb lokális rendszerébe + Alkalmazzuk a Body IK-t
        lx, ly, lz, global_pos = self.kinematics.body_to_leg_coords(
            target_x, target_y, target_z, leg_cfg, self.body_rpy
        )
        
        # Vizualizáció (opcionális, csak az 1-es lábnál)
        # A globális, elforgatott pontot rajzoljuk ki
        if leg_key == "leg_1":
            self.publish_marker_global(global_pos[0], global_pos[1], global_pos[2])

        # 2. Kiszámoljuk a szögeket
        return self.kinematics.compute_ik(lx, ly, lz)

    def timer_callback(self):
        # 1. Ramping
        self.current_vel['x'] = self.ramp_value(self.current_vel['x'], self.cmd_vel['x'], self.ramp_step)
        self.current_vel['y'] = self.ramp_value(self.current_vel['y'], self.cmd_vel['y'], self.ramp_step)
        self.current_vel['yaw'] = self.ramp_value(self.current_vel['yaw'], self.cmd_vel['yaw'], self.ramp_step)

        # 2. Debug Pub
        d_msg = Twist()
        d_msg.linear.x = self.current_vel['x']
        d_msg.linear.y = self.current_vel['y']
        d_msg.angular.z = self.current_vel['yaw']
        self.debug_pub.publish(d_msg)

        # 3. Demo Mód (Lélegzés / Breathing)
        now = time.time()
        t = now - self.start_time
        
        if abs(self.cmd_vel['x']) < 0.01 and abs(self.cmd_vel['y']) < 0.01 and abs(self.cmd_vel['yaw']) < 0.01:
            # 3 másodperces ciklus: 2s mozgás, 1s pihenés
            cycle_time = 3.0
            t_cycle = t % cycle_time
            
            if t_cycle < 2.0:
                # Egy sima szinusz hullám, ami 0-ról indul, felmegy a csúcsra, majd visszatér 0-ra
                # A -15.0 a mozgás amplitúdója milliméterben (mivel a Z lefelé pozitív a lábhoz képest, a negatív érték emeli a testet)
                self.breathe_z = math.sin((t_cycle / 2.0) * math.pi) * 0.0
            else:
                # 1 másodperc várakozás alaphelyzetben
                self.breathe_z = 0.0
                
            # --- ÚJ: SZOFTVERES FENEKEMELÉS (ÁLLÓ HELYZETBEN) ---
            # A pitch a bólogatás. -5.0 fok általában előre dönti (emeli a hátulját).
            # Ha nálad pont az orrát emelné fel, akkor írd át +5.0-re!
            self.body_rpy = {'roll': 0.0, 'pitch': math.radians(-5.0), 'yaw': 0.0}
        else:
            self.breathe_z = 0.0
            # --- ÚJ: SZOFTVERES FENEKEMELÉS (SÉTA KÖZBEN IS) ---
            self.body_rpy = {'roll': 0.0, 'pitch': math.radians(-5.0), 'yaw': 0.0}

        # 4. Lábak mozgatása
        all_joints = {}
        for leg_key, leg_cfg in self.LEGS.items():
            coxa, femur, tibia = self.process_leg(leg_key, leg_cfg, t)
            all_joints[leg_key] = [coxa, femur, tibia]

        self.publish_joints_all(all_joints)

    # ... PUBLISHEREK ...

    def publish_marker_global(self, x, y, z):
        msg = Marker()
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.type = Marker.SPHERE; msg.action = Marker.ADD
        msg.pose.position.x = x/1000.0; msg.pose.position.y = y/1000.0; msg.pose.position.z = z/1000.0
        msg.pose.orientation.w = 1.0; msg.scale.x = 0.03; msg.scale.y = 0.03; msg.scale.z = 0.03
        msg.color.r, msg.color.g, msg.color.b, msg.color.a = 0.0, 1.0, 0.0, 1.0
        self.marker_pub.publish(msg)

    def publish_joints_all(self, joint_map):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = []; msg.position = []
        for leg_key, angles in joint_map.items():
            try: lid = leg_key.split('_')[1]
            except: continue
            msg.name.extend([f'joint_{lid}_coxa', f'joint_{lid}_femur', f'joint_{lid}_tibia'])
            msg.position.extend(angles)
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HexapodController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()