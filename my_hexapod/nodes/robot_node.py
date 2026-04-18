import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist, Point
from rcl_interfaces.msg import SetParametersResult
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String, ColorRGBA
from visualization_msgs.msg import Marker
import json
import time
import math
import os

# A TF2 Broadcaster a térbeli hídhoz
from tf2_ros import TransformBroadcaster

# === IMPORTÁLJUK A MODULOKAT ===
from my_hexapod.core.robot_kinematics import HexapodKinematics 
from my_hexapod.core.robot_gait import HexapodGait 
from my_hexapod.core.robot_odometry import HexapodOdometry 
from my_hexapod.core.robot_visualizer import HexapodVisualizer

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
        self.odometry = HexapodOdometry()
        
        self.kinematics = HexapodKinematics(self.config)
        self.gait = HexapodGait()
        self.odometry = HexapodOdometry()
        self.visualizer = HexapodVisualizer() # <-- ÚJ!

        # 3. ÁLLAPOTVÁLTOZÓK
        self.cmd_vel = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
        self.current_vel = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
        self.ramp_step = 0.05  #Gyorsulás lépésenként (0.01 = 1% gyorsulás minden ciklusban)
        
        self.body_rpy = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.breathe_z = 0.0
        self.anim_leg_offsets = {}
        
        # A Főnöktől kapott hivatalos állapot
        self.robot_state_str = "IDLE" 
        
        # Virtuális idő a lopakodó járáshoz
        self.virtual_walk_time = 0.0
        
        # 4. DINAMIKUS PARAMÉTEREK DEKLARÁLÁSA
        self.declare_parameters(
            namespace='',
            parameters=[
                ('gait_freq', 1.6),
                ('gait_step_len', 120.0),
                ('gait_step_height', 80.0),
                ('gait_base_dist', 280.0),
                ('gait_base_height', -100.0)
            ]
        )

        # Kezdeti értékek átadása a gait modulnak
        self.gait.params['freq'] = self.get_parameter('gait_freq').value
        self.gait.params['step_len'] = self.get_parameter('gait_step_len').value
        self.gait.params['step_height'] = self.get_parameter('gait_step_height').value
        self.gait.params['base_dist'] = self.get_parameter('gait_base_dist').value
        self.gait.params['base_height'] = self.get_parameter('gait_base_height').value

        # Feliratkozás a változásokra (Ha a Foxglove-ban átírod, ez a függvény hívódik meg)
        self.add_on_set_parameters_callback(self.parameters_callback)

        # 5. ROS KOMMUNIKÁCIÓ
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 1)
        self.debug_pub = self.create_publisher(Twist, 'current_vel', 1)
        
        # VAK ODOMETRIA (KONTROLLCSOPORT)
        self.odom_pub = self.create_publisher(Odometry, 'odom/kinematic', 10)
        self.path_pub = self.create_publisher(Path, 'odom/kinematic_path', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # Már nem az animációkat, hanem a tiszta robot_state-et figyeljük az Agytól!
        self.state_sub = self.create_subscription(String, 'robot_state', self.state_callback, 10)
        
        # IMU Feliratkozás és változók az Auto-Levelinghez
        self.imu_sub = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)
        
        # Ezekben tároljuk a valós, szűrt fizikai dőlést (radiánban)
        self.imu_roll = 0.0
        self.imu_pitch = 0.0
        
        # IMU KALIBRÁCIÓ ÉS BEÁLLÍTÁSOK
        # Memória helyett mostantól csak simítjuk (EMA filter) az aktuális célpozíciót
        self.leveling_roll_smoothed = 0.0
        self.leveling_pitch_smoothed = 0.0
        
        # Milyen erősen reagáljon a dőlésre? (1.0 = 100% kompenzáció, 0.5 = 50%)
        # Kezdd 0.8-cal, ha túllő (oszcillál), vedd le 0.6-ra!
        self.kp_leveling = 0.8
        
        # Milyen gyors legyen a simítás? (1.0 = azonnali rántás, 0.1 = lágy úsztatás)
        self.alpha_leveling = 0.2
        
        # 1. TENGELY INVERZIÓ: Ha valamelyik tengelyen ráhajt a dőlésre korrigálás helyett, 
        # írd át 1.0-ról -1.0-ra!
        self.leveling_roll_dir = -1.0  
        self.leveling_pitch_dir = -1.0 
        
        # 2. SZENZOR OFFSZET: Ha egyenes asztalon is elcsavarja magát, 
        # itt megadhatod a szenzor alap fizikai tévedését radiánban.
        self.imu_roll_offset = -0.0349
        self.imu_pitch_offset = 0.0
        
        # --- ÚJ: BEKAPCSOLÁSI KÉSLELTETÉS ---
        self.imu_warmup_time = 3.0 # 3 másodperc türelem
        self.robot_start_time = time.time()
        
        # Lábvégek (mancsok) publikálása a Foxglove 3D-hez
        self.foot_tips_pub = self.create_publisher(Marker, 'debug/foot_tips', 10)
        
        self.timer = self.create_timer(1.0 / 50.0, self.timer_callback)
        self.start_time = time.time()
        
        self.get_logger().info("Hexapod Controller: Okosított Izomzat üzemmód (TF2-vel) aktív! 💪")

    def cmd_vel_callback(self, msg):
        self.cmd_vel['x'] = msg.linear.x
        self.cmd_vel['y'] = msg.linear.y
        self.cmd_vel['z'] = msg.linear.z  
        self.cmd_vel['yaw'] = msg.angular.z
        
    def parameters_callback(self, params):
        """ Ez fut le azonnal, amint a Foxglove-ban átállítasz egy értéket """
        for param in params:
            if param.name == 'gait_freq':
                self.gait.params['freq'] = param.value
            elif param.name == 'gait_step_len':
                self.gait.params['step_len'] = param.value
            elif param.name == 'gait_step_height':
                self.gait.params['step_height'] = param.value
            elif param.name == 'gait_base_dist':
                self.gait.params['base_dist'] = param.value
            elif param.name == 'gait_base_height':
                self.gait.params['base_height'] = param.value
        return SetParametersResult(successful=True)

    def state_callback(self, msg):
        """ Fogadja a hivatalos Állapotot a Főnöktől (Pl. 'WALK_RIPPLE' vagy 'ANIM_ATTACK') """
        self.robot_state_str = msg.data
        
    def imu_callback(self, msg):
        """ Fogadja a szűrt IMU adatokat és kiszámolja belőle a dőlésszöget """
        q = msg.orientation
        
        # 1. Roll (X tengely körüli dőlés) számítása kvaternióból
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        raw_roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # 2. Pitch (Y tengely körüli dőlés) számítása kvaternióból
        sinp = 2 * (q.w * q.y - q.z * q.x)
        # Matematikai biztonsági korlát (ha a szenzor pont 90 fokban állna)
        if abs(sinp) >= 1:
            raw_pitch = math.copysign(math.pi / 2, sinp) 
        else:
            raw_pitch = math.asin(sinp)
            
        # Egy minimális szoftveres aluláteresztő szűrő (Low-Pass Filter), 
        # hogy a hirtelen rántásokra ne reagáljon agresszívan a robot.
        alpha = 0.2
        self.imu_roll = (1.0 - alpha) * self.imu_roll + (alpha * raw_roll)
        self.imu_pitch = (1.0 - alpha) * self.imu_pitch + (alpha * raw_pitch)

    def ramp_value(self, current, target, step):
        """ 
        Konstans gyorsulás (Linear Ramping). 
        A legkíméletesebb a szervóknak, mert az erőkifejtés (nyomaték) állandó.
        """
        diff = target - current
        if abs(diff) < step:
            return target
        if diff > 0:
            return current + step
        else:
            return current - step

    def process_leg(self, leg_key, leg_cfg, t):
        """ Egy láb teljes feldolgozása a modulok segítségével """
        
        # A. JÁRÁS (Gait Module)
        phase = self.gait.get_leg_phase(leg_key, t)
        
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

        # Lekérjük az esetleges egyéni láb animációt (pl. integetés)
        anim_off = self.anim_leg_offsets.get(leg_key, {'x': 0.0, 'y': 0.0, 'z': 0.0})

        # Végső CÉLPONT (Test koordinátarendszerben)
        # Ez az a pont a térben, ahova a láb végét tenni akarjuk (Body IK nélkül)
        target_x = neutral_x + off_walk + dx_rot + anim_off['x']
        target_y = neutral_y + off_strafe + dy_rot + anim_off['y']
        
        # Sima, időalapú Z magasság a szinuszgörbéből és a lélegzésből
        target_z = self.gait.params['base_height'] + off_z + self.breathe_z + anim_off['z']

        # C. KINEMATIKA (Kinematics Module)
        # 1. Transzformáljuk a pontot a láb lokális rendszerébe + Alkalmazzuk a Body IK-t
        lx, ly, lz, global_pos = self.kinematics.body_to_leg_coords(
            target_x, target_y, target_z, leg_cfg, self.body_rpy
        )

        # 2. Kiszámoljuk a szögeket
        angles = self.kinematics.compute_ik(lx, ly, lz)
        return angles, global_pos

    def timer_callback(self):
        # 1. Ramping (A szervók mechanikai kímélése)
        self.current_vel['x'] = self.ramp_value(self.current_vel['x'], self.cmd_vel['x'], self.ramp_step)
        self.current_vel['y'] = self.ramp_value(self.current_vel['y'], self.cmd_vel['y'], self.ramp_step)
        self.current_vel['z'] = self.ramp_value(self.current_vel['z'], self.cmd_vel['z'], self.ramp_step) 
        self.current_vel['yaw'] = self.ramp_value(self.current_vel['yaw'], self.cmd_vel['yaw'], self.ramp_step)

        # 2. ENGEDELMESKEDÉS A FŐNÖKNEK
        now = time.time()
        t = now - self.start_time
        dt = 1.0 / 50.0  # Mivel 50Hz-en fut a timer
        
        base_h = self.get_parameter('gait_base_height').value
        base_step_h = self.get_parameter('gait_step_height').value
        
        # Kiszámoljuk az eltolást (current_vel['z'] -1.0 és 1.0 között mozog)
        # A testen 40mm-t emelünk/süllyesztünk max.
        z_offset_mm = self.current_vel['z'] * 40.0 
        self.gait.params['base_height'] = base_h + z_offset_mm
        
        # --- ÚJ: A lépésmagasság követi a testmagasságot ---
        # Ha a testet emeljük (Z > 0), a lépést is emeljük (pl. max +40mm-rel)
        # Ha süllyesztjük, a lépés is kisebb lesz.
        # A max(10.0, ...) biztosítja, hogy ne legyen 0 vagy negatív a lépés, különben megbotlik.
        step_offset_mm = self.current_vel['z'] * 40.0
        self.gait.params['step_height'] = max(20.0, base_step_h - step_offset_mm)
        
        # Szétszedjük a parancsot
        state_parts = self.robot_state_str.split('_', 1)
        main_state = state_parts[0] 
        sub_state = state_parts[1] if len(state_parts) > 1 else None

        # LOPAKODÓ LOGIKA (Sebességmérés)
        mag = math.sqrt(self.current_vel['x']**2 + self.current_vel['y']**2 + self.current_vel['yaw']**2)
        mag = min(1.0, max(0.0, mag)) # Limitáljuk 0.0 és 1.0 közé

        if main_state == "WALK":
            
            # ÚJ: FÁZIS KÖZÉPRE IGAZÍTÁSA INDULÁSKOR (A hátrafelé rántás ellen)
            if self.gait.current_state != "WALK":
                self.gait.gait_mode = sub_state if sub_state else "TRIPOD"
                
                # Lekérjük az aktuális járásmód talaj-arányát
                push_fraction, _ = self.gait._get_gait_profile()
                
                # Beállítjuk az időt úgy, hogy az alap fázis pontosan a húzás közepén (0.5) induljon!
                self.virtual_walk_time = (push_fraction / 2.0) / self.gait.params['freq']
                
            self.gait.current_state = "WALK"
            self.gait.active_animation = None
            
            if mag > 0.01:
                speed_factor = 0.25 + (0.75 * mag)
                self.virtual_walk_time += (dt * speed_factor)

        elif main_state == "ANIM":
            self.gait.current_state = "ANIMATION"
            self.gait.active_animation = sub_state
        else: # IDLE parancs érkezett
            if mag > 0.01:
                self.gait.current_state = "WALK"
                # IDLE-ből is lassan álljon meg, ha még van lendület
                speed_factor = 0.25 + (0.75 * mag)
                self.virtual_walk_time += (dt * speed_factor)
            else:
                self.gait.current_state = "IDLE"
                self.gait.active_animation = None
            
        # Lekérjük az "ideális" dőlést, amit az animáció vagy a járás kér
        self.body_rpy, self.breathe_z, self.anim_leg_offsets = self.gait.get_body_pose(t)

        if self.gait.current_state != "ANIMATION":
            now = time.time()
            
            # --- 0. BIZTONSÁGI VÁRAKOZÁS ---
            # Várjuk meg, amíg a Madgwick szűrő magához tér a bekapcsolás után!
            if (now - self.robot_start_time) < self.imu_warmup_time:
                # Ezalatt az idő alatt a robot ne kompenzáljon semmit.
                pass 
            else:
                # 1. Kiszámoljuk a nyers hibát
                raw_error_roll = 0.0 - (self.imu_roll - self.imu_roll_offset)
                raw_error_pitch = 0.0 - (self.imu_pitch - self.imu_pitch_offset)
                
                # 2. Irányok alkalmazása
                error_roll = raw_error_roll * self.leveling_roll_dir
                error_pitch = raw_error_pitch * self.leveling_pitch_dir
                
                # 3. HOLTSÁV (Deadband): +/- 1.5 fokon belül (0.026 rad) nem csinál semmit.
                deadband = math.radians(1.5)
                if abs(error_roll) < deadband: error_roll = 0.0
                if abs(error_pitch) < deadband: error_pitch = 0.0

                # 4. PROPORCIONÁLIS SZABÁLYOZÁS (Azonnali reakció!)
                target_comp_roll = error_roll * self.kp_leveling
                target_comp_pitch = error_pitch * self.kp_leveling
                
                # 5. BIZTONSÁGI KORLÁT (Clamping): Max 20 fok
                max_comp = math.radians(20.0) 
                target_comp_roll = max(-max_comp, min(max_comp, target_comp_roll))
                target_comp_pitch = max(-max_comp, min(max_comp, target_comp_pitch))

                # 6. SIMÍTÁS (EMA Filter) - Hogy ne rángasson agresszívan
                self.leveling_roll_smoothed = (1.0 - self.alpha_leveling) * self.leveling_roll_smoothed + (self.alpha_leveling * target_comp_roll)
                self.leveling_pitch_smoothed = (1.0 - self.alpha_leveling) * self.leveling_pitch_smoothed + (self.alpha_leveling * target_comp_pitch)
                
                # 7. ALKALMAZÁS A TESTRE
                self.body_rpy['roll'] += self.leveling_roll_smoothed
                self.body_rpy['pitch'] += self.leveling_pitch_smoothed

        # --- 3. ODOMETRIA ÉS TF HÍVÁSA AZ ÚJ MODULBÓL ---
        is_walking = (self.gait.current_state == "WALK")
        odom_msg, tf_msg, path_msg, path_updated = self.odometry.update_and_generate_messages(
            self.current_vel['x'], self.current_vel['y'], self.current_vel['yaw'],
            self.gait.params['freq'], self.gait.params['step_len'], self.gait.params['base_dist'],
            is_walking, self.get_clock().now().to_msg()
        )

        # --- ÚJ: BIZONYTALANSÁG (KOVARIANCIA) BEÁLLÍTÁSA ---
        # Ráerőltetjük a generált üzenetre a bizonytalanságot, mielőtt kimenne a hálózatra!
        odom_msg.twist.covariance[0] = 0.05   # X tengelyű sebesség bizonytalansága
        odom_msg.twist.covariance[7] = 0.05   # Y tengelyű sebesség bizonytalansága
        odom_msg.twist.covariance[35] = 0.05  # Forgási sebesség bizonytalansága

        # Publikálás a hálózatra
        self.odom_pub.publish(odom_msg)
        #self.tf_broadcaster.sendTransform(tf_msg)
        if path_updated:
            self.path_pub.publish(path_msg)

        # 4. KINEMATIKA ÉS VIZUALIZÁCIÓ KISZÁMÍTÁSA (Markerek)
        
        all_joints = {}
        foot_positions_for_debug = [] # Ide gyűjtjük a koordinátákat a zöld golyóknak

        for leg_key, leg_cfg in self.LEGS.items():
            # Lábak kiszámítása
            angles, global_pos = self.process_leg(leg_key, leg_cfg, self.virtual_walk_time)
            all_joints[leg_key] = angles
            foot_positions_for_debug.append(global_pos)

        # Hardver parancs kiküldése (Ettől mozog a fizikai robot!)
        self.publish_joints_all(all_joints)

        # Foxglove kozmetika kiküldése (Az új vizualizációs modulon keresztül)
        marker_msg = self.visualizer.generate_foot_tips_marker(
            foot_positions_for_debug, 
            self.get_clock().now().to_msg()
        )
        self.foot_tips_pub.publish(marker_msg)

    # ... PUBLISHEREK ...
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