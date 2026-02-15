import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist
import json, os, time, math

class HexapodController(Node):
    def __init__(self):
        super().__init__('hexapod_controller')
        
        # --- 1. KONFIGURÁCIÓ (JSON) ---
        script_dir = os.path.dirname(os.path.realpath(__file__))    # Munkstatárhely meghatározása
        config_path = os.path.join(script_dir, 'robot_config.json') # Konfig fájl elérési útja
        with open(config_path, 'r') as f:
            self.config = json.load(f)
            
        dims = self.config['dimensions']
        self.coxaLength = dims['coxa_length']
        self.femurZOffset = dims['femur_z_offset']
        self.z_offset = dims['base_z_offset']
        
        # Femur & Tibia Matek (Előszámolva)
        self.femur_x = dims['femur_x']
        self.femur_drop = dims['femur_drop']
        self.femurLength_eff = math.sqrt(self.femur_x**2 + self.femur_drop**2)
        self.femur_angle_offset = math.atan2(self.femur_drop, self.femur_x)
        
        self.tibia_x = dims['tibia_x']
        self.tibia_drop = dims['tibia_drop']
        self.tibiaLength_eff = math.sqrt(self.tibia_x**2 + self.tibia_drop**2)
        self.tibia_angle_offset = math.atan2(self.tibia_drop, self.tibia_x)

        self.LEGS = self.config['legs']

        # --- 2. MOZGÁS ÁLLAPOT (STATE) ---
        # Ez a rész felel majd az irányításért (Teleop)
        # Most fix értékeket adunk neki, később a billentyűzet ezt írja felül.
        self.cmd_vel = {
            'x': 0.0,  # Előre sebesség (0.0 = áll, 1.0 = teljes sebesség)
            'y': 0.0,  # Oldalazás (később)
            'yaw': 0.0 # Forgás (később)
        }
        
        # VALÓS SEBESSÉG (Ami "üldözi" a parancsot)
        self.current_vel = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        
        # RAMPING PARAMÉTER (Gyorsulás mértéke)
        # Mennyit változhat a sebesség egy ciklus alatt? (0.01 - 0.1 között jó)
        # Kisebb szám = lassabb gyorsulás (simább mozgás)
        self.ramp_step = 0.05
        
        # Test Orientáció Radiánban(Body IK)
        self.body_rpy = {
            'roll': 0.0,  # Dőlés jobbra-balra (X tengely körül)
            'pitch': 0.0, # Bólintás előre-hátra (Y tengely körül)
            'yaw': 0.0    # Test forgatás (Z tengely körül) - Ez független a járástól!
        }
        
        # Járás paraméterek
        self.gait_params = {
            'freq': 4.0,           # Sebesség (Hz)
            'step_len': 40.0,      # Lépéshossz (mm)
            'step_height': 30.0,   # Lépésmagasság (mm)
            'base_dist': 220.0,    # Alap terpesz
            'base_height': -100.0  # Test magasság
        }

        # --- 3. ROS KOMMUNIKÁCIÓ ---
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 1)
        self.marker_pub = self.create_publisher(Marker, 'target_marker', 1)
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        # ÚJ: DEBUG PUBLISHER (Ezen küldjük ki a simított sebességet)
        self.debug_pub = self.create_publisher(Twist, 'current_vel', 1)
        
        self.timer = self.create_timer(1.0 / 25.0, self.timer_callback)
        self.start_time = time.time()
        
        self.get_logger().info("Hexapod Controller: Refactored & Ready!")

    # === SEGÉDFÜGGVÉNYEK ===

    def body_to_leg_coords(self, x_body, y_body, z_leg, leg_config):
        # 1. ELTOLÁS
        dx = x_body - leg_config["mount_x"]
        dy = y_body - leg_config["mount_y"]
        
        # 2. FORGATÁS (Már radiánban van, nem kell átváltani!)
        theta = leg_config["mount_angle_rad"]
        
        # Forgatási mátrix
        x_leg = dx * math.cos(theta) + dy * math.sin(theta)
        y_leg = -dx * math.sin(theta) + dy * math.cos(theta)
        
        return x_leg, y_leg, z_leg

    def compute_ik(self, x, y, z):
        # 1. ALAPSZÖG (Coxa elfordulása)
        # Ez marad a régi, mert a Coxa a vállból forog a cél felé.
        fi = math.atan2(y, x)

        # 2. SÍKBELI TÁVOLSÁGOK KORRIGÁLÁSA
        # A teljes távolság a válltól a célig (madártávlatból):
        R_total = math.sqrt(x**2 + y**2)
        
        # Ebből le kell vonni a Coxa hosszát!
        # Mert a Femur-Tibia háromszög csak a Coxa vége után kezdődik.
        R_effective = R_total - self.coxaLength
        
        # 3. MAGASSÁG KORRIGÁLÁSA
        # Az URDF szerint a Femur 15mm-rel feljebb/lejjebb kezdődik, mint a Coxa. Ezt is bele kell számolni a háromszög magasságába.
        # Ha az URDF-ben z=0.015 pozitív, akkor a Femur "fentebb" van, tehát a célpont "mélyebben" van hozzá képest.
        actual_z = (self.z_offset - z) + self.femurZOffset

        # 4. A HÁROMSZÖG ÁTFOGÓJA (D)
        # Most már a korrigált távolságokkal számolunk!
        D = math.sqrt(R_effective**2 + actual_z**2)

        # Biztonsági limit (itt is a korrigált D-t figyeljük)
        max_reach = self.femurLength_eff + self.tibiaLength_eff
        if D > max_reach:
            D = max_reach - 0.001

        # === INNENTŐL UGYANAZ A MATEK ===
        # Gamma (emelkedés) a korrigált távolságokkal
        # Figyelem: Itt az R_effective-et használjuk, nem az R_total-t!
        gamma = math.atan2(actual_z, R_effective)

        # Koszinusz tétel (változatlan)
        val_alpha = (self.femurLength_eff**2 + D**2 - self.tibiaLength_eff**2) / (2 * self.femurLength_eff * D)
        alpha = math.acos(max(min(val_alpha, 1.0), -1.0))
        
        val_beta = (self.tibiaLength_eff**2 + self.femurLength_eff**2 - D**2) / (2 * self.femurLength_eff * self.tibiaLength_eff)
        beta  = math.acos(max(min(val_beta, 1.0), -1.0))

        # Kimenet
        fi_rad = fi
        alpha_rad = (gamma - alpha) - self.femur_angle_offset
        beta_rad = (math.pi - beta) - self.tibia_angle_offset

        return fi_rad, alpha_rad, beta_rad

    def get_tripod_phase(self, leg_key, t):
        """ Kiszámolja a láb aktuális fázisát (0..2PI) az idő és a csoport alapján. """
        # Tripod csoportosítás
        group_a = ["leg_1", "leg_3", "leg_5"]
        
        phase_offset = 0.0
        if leg_key not in group_a:
            phase_offset = math.pi # B csoport ellentétes fázisban
            
        return (t * self.gait_params['freq']) + phase_offset

    def get_sine_trajectory(self, phase, velocity_x):
        """ 
        Generálja a D-betű pályát. 
        phase: Az aktuális szög a ciklusban
        velocity_x: Milyen gyorsan akarunk menni (-1.0 ... 1.0)
        """
        # Ha nincs sebesség, nincs oszcilláció (Alapállás)
        if abs(velocity_x) < 0.01:
            return 0.0, 0.0

        # X irány (Előre-Hátra)
        # step_len * velocity (hogy tudjunk lassítani/tolatni)
        amplitude_x = (self.gait_params['step_len'] / 2.0) * velocity_x
        off_x = -math.cos(phase) * amplitude_x
        
        # Z irány (Emelés)
        raw_sine = math.sin(phase)
        off_z = 0.0
        
        # Csak akkor emelünk, ha előre megyünk (és a sebesség iránya is előre mutat)
        # VAGY: Egyszerűsített logika -> A szinusz pozitív felében mindig emelünk
        if raw_sine > 0:
            off_z = raw_sine * self.gait_params['step_height']
            
        return off_x, off_z

    # === TEST KINEMATIKA SEGÉDFÜGGVÉNYEK ===
    
    def rotate_x(self, x, y, z, angle):
        """ Roll: Forgatás az X tengely körül (Y és Z változik) """
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        y_new = y * cos_a - z * sin_a
        z_new = y * sin_a + z * cos_a
        return x, y_new, z_new

    def rotate_y(self, x, y, z, angle):
        """ Pitch: Forgatás az Y tengely körül (X és Z változik) """
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        x_new = x * cos_a + z * sin_a
        z_new = -x * sin_a + z * cos_a
        return x_new, y, z_new

    def rotate_z(self, x, y, z, angle):
        """ Yaw: Forgatás a Z tengely körül (X és Y változik) """
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        x_new = x * cos_a - y * sin_a
        y_new = x * sin_a + y * cos_a
        return x_new, y_new, z

    def process_leg(self, leg_key, leg_cfg, t):
        # 1. Fázis és Vezérlés (Ez marad a régi)
        phase = self.get_tripod_phase(leg_key, t)
        
        # 2. Most már a SIMÍTOTT sebességet használjuk a matekhoz!
        vel_x = self.current_vel['x']      
        vel_y = self.current_vel['y']      
        vel_yaw = self.current_vel['yaw']  
        
        # FONTOS: A mozgás állapotát is a valós sebesség alapján döntjük el!
        # Ha elengeded a gombot (cmd=0), a robot még lassul (current > 0), tehát mozognia kell.
        is_moving = (abs(vel_x) > 0.001 or abs(vel_y) > 0.001 or abs(vel_yaw) > 0.001)

        # 2. Lépés Generálás (Séta + Oldalazás + Forgás) (Ez is marad)
        walk_amp = (self.gait_params['step_len'] / 2.0) * vel_x
        off_walk = -math.cos(phase) * walk_amp

        strafe_amp = (self.gait_params['step_len'] / 2.0) * vel_y
        off_strafe = -math.cos(phase) * strafe_amp

        turn_amp = (self.gait_params['step_len'] / 2.0) * vel_yaw
        off_turn = -math.cos(phase) * turn_amp

        off_z = 0.0
        if is_moving and math.sin(phase) > 0:
            off_z = math.sin(phase) * self.gait_params['step_height']

        # 3. Alap Koordináták
        angle = leg_cfg["mount_angle_rad"]
        dist = self.gait_params['base_dist']
        
        neutral_x = dist * math.cos(angle)
        neutral_y = dist * math.sin(angle)

        dx_rot = (-neutral_y / dist) * off_turn
        dy_rot = (neutral_x / dist) * off_turn

        # --- EDDIG MINDEN UGYANAZ VOLT ---
        
        # 4. A Láb Pozíciója a TESTHEZ képest (Rotáció nélkül)
        # Ez az a pont, ahol a láb a földön van (vagy a levegőben lép)
        foot_x = neutral_x + off_walk + dx_rot
        foot_y = neutral_y + off_strafe + dy_rot
        foot_z = self.gait_params['base_height'] + off_z

        # 5. BODY IK: TEST FORGATÁS ALKALMAZÁSA 🥋
        # A trükk: Ha a test dől, az olyan, mintha a láb koordinátáit elforgatnánk.
        
        rx, ry, rz = foot_x, foot_y, foot_z
        
        # Fontos: A sorrend számít! Általában: Yaw -> Pitch -> Roll
        rx, ry, rz = self.rotate_z(rx, ry, rz, self.body_rpy['yaw'])
        rx, ry, rz = self.rotate_y(rx, ry, rz, self.body_rpy['pitch'])
        rx, ry, rz = self.rotate_x(rx, ry, rz, self.body_rpy['roll'])
        
        # A végeredmény a transzformált láb pozíció
        final_x, final_y, final_z = rx, ry, rz

        # 6. Vizualizáció (Csak az 1-es lábnál)
        if leg_key == "leg_1":
            self.publish_marker_global(final_x, final_y, final_z)

        # 7. Inverz Kinematika
        lx, ly, lz = self.body_to_leg_coords(final_x, final_y, final_z, leg_cfg)
        return self.compute_ik(lx, ly, lz)

    def cmd_vel_callback(self, msg):
        """
        Ez a függvény hívódik meg, ha érkezik egy Twist üzenet (pl. billentyűzetről).
        msg.linear.x -> Előre/Hátra sebesség
        msg.linear.y -> Oldalazási sebesség (Strafe)
        msg.angular.z -> Forgási sebesség (Yaw)
        """
        # Frissítjük a belső állapotot
        self.cmd_vel['x'] = msg.linear.x
        self.cmd_vel['y'] = msg.linear.y
        self.cmd_vel['yaw'] = msg.angular.z
        
    def ramp_value(self, current, target, step):
        """ Finoman közelíti a jelenlegi értéket a célhoz """
        if current < target:
            return min(current + step, target)
        elif current > target:
            return max(current - step, target)
        return target

    # === FŐ CIKLUS ===

    def timer_callback(self):
        
        self.current_vel['x'] = self.ramp_value(self.current_vel['x'], self.cmd_vel['x'], self.ramp_step)
        self.current_vel['y'] = self.ramp_value(self.current_vel['y'], self.cmd_vel['y'], self.ramp_step)
        self.current_vel['yaw'] = self.ramp_value(self.current_vel['yaw'], self.cmd_vel['yaw'], self.ramp_step)

        # --- ÚJ: DEBUG ADAT KÜLDÉSE ---
        debug_msg = Twist()
        debug_msg.linear.x = self.current_vel['x']
        debug_msg.linear.y = self.current_vel['y']
        debug_msg.angular.z = self.current_vel['yaw']
        self.debug_pub.publish(debug_msg)
        # ------------------------------
        
        # Most már nagyon tiszta a fő ciklus!
        now = time.time()
        t = now - self.start_time
        
        # DEMO MÓD: Ha a robot áll, "lélegezzen" (Body IK Demo)
        # Ha megmozdulsz a billentyűzettel, ez megáll.
        if abs(self.cmd_vel['x']) < 0.01 and abs(self.cmd_vel['y']) < 0.01 and abs(self.cmd_vel['yaw']) < 0.01:
            # Finom dőlés (Pitch és Roll) szinusz hullámokkal
            self.body_rpy['pitch'] = math.sin(t * 2.0) * 0.15  # Kb 8 fok előre-hátra
            self.body_rpy['roll']  = math.cos(t * 2.0) * 0.15  # Kb 8 fok jobbra-balra
            self.body_rpy['yaw']   = 0.0
        else:
            # Ha járunk, tartsuk egyenesen a testet (vagy nullázzuk)
            self.body_rpy['pitch'] = 0.0
            self.body_rpy['roll']  = 0.0
            self.body_rpy['yaw']   = 0.0
        
        all_joints = {}

        # Csak végigiterálunk a lábakon, a nehéz munkát a process_leg végzi
        for leg_key, leg_cfg in self.LEGS.items():
            coxa, femur, tibia = self.process_leg(leg_key, leg_cfg, t)
            all_joints[leg_key] = [coxa, femur, tibia]

        self.publish_joints_all(all_joints)

    # ... PUBLISHEREK ...

    def publish_marker_global(self, x, y, z):
        msg = Marker()
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.type = Marker.SPHERE
        msg.action = Marker.ADD
        # Nincs trükk, nincs eltolás, ez a nyers célpont
        msg.pose.position.x = x / 1000.0
        msg.pose.position.y = y / 1000.0
        msg.pose.position.z = z / 1000.0
        msg.pose.orientation.w = 1.0
        msg.scale.x, msg.scale.y, msg.scale.z = 0.03, 0.03, 0.03
        msg.color.r, msg.color.g, msg.color.b, msg.color.a = 0.0, 1.0, 0.0, 1.0
        self.marker_pub.publish(msg)

    def publish_joints_all(self, joint_map):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = []
        msg.position = []
        
        for leg_key, angles in joint_map.items():
            try:
                leg_id = leg_key.split('_')[1] 
            except IndexError: continue

            msg.name.extend([f'joint_{leg_id}_coxa', f'joint_{leg_id}_femur', f'joint_{leg_id}_tibia'])
            msg.position.extend([angles[0], angles[1], angles[2]]) # Már radián!

        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HexapodController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()