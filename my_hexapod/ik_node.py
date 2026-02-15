import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
import json, os, time, math

class HexapodController(Node):
    def __init__(self):
        super().__init__('hexapod_controller')
        
        # 1. KONFIGURÁCIÓ BETÖLTÉSE JSON-BŐL
        # Megkeressük, hol van EZ a python fájl a lemezen és hozzáfűzzük a robot_config.json elérési útját
        script_dir = os.path.dirname(os.path.realpath(__file__))
        config_path = os.path.join(script_dir, 'robot_config.json')

        with open(config_path, 'r') as f:
            self.config = json.load(f)
            
        dims = self.config['dimensions']
        
        # 2. MÉRETEK KISZEDÉSE ÉS ELŐSZÁMÍTÁS
        self.coxaLength = dims['coxa_length']
        self.femurZOffset = dims['femur_z_offset']
        self.z_offset = dims['base_z_offset']
        
        # Femur matek
        self.femur_x = dims['femur_x']
        self.femur_drop = dims['femur_drop']
        self.femurLength_eff = math.sqrt(self.femur_x**2 + self.femur_drop**2)
        self.femur_angle_offset = math.atan2(self.femur_drop, self.femur_x)
        
        # Tibia matek
        self.tibia_x = dims['tibia_x']
        self.tibia_drop = dims['tibia_drop']
        self.tibiaLength_eff = math.sqrt(self.tibia_x**2 + self.tibia_drop**2)
        self.tibia_angle_offset = math.atan2(self.tibia_drop, self.tibia_x)

        # Lábak konfigurációja (közvetlenül a JSON-t használjuk)
        self.LEGS = self.config['legs']

        # 3. ROS INICIALIZÁLÁS
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 1)
        self.marker_pub = self.create_publisher(Marker, 'target_marker', 1)
        
        # Frissítési ráta
        self.timer = self.create_timer(1.0 / 25.0, self.timer_callback)
        self.start_time = time.time()
        
        self.get_logger().info("Hexapod Controller: JSON Config Loaded & Ready!")

    def body_to_leg_coords(self, x_body, y_body, z_body, leg_config):
        # 1. ELTOLÁS
        dx = x_body - leg_config["mount_x"]
        dy = y_body - leg_config["mount_y"]
        
        # 2. FORGATÁS (Már radiánban van, nem kell átváltani!)
        theta = leg_config["mount_angle_rad"]
        
        # Forgatási mátrix
        x_leg = dx * math.cos(theta) + dy * math.sin(theta)
        y_leg = -dx * math.sin(theta) + dy * math.cos(theta)
        
        z_leg = z_body 
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

    def publish_marker_global(self, x, y, z):
        """ A zöld gömböt most már a TESTHEZ (base_link) képest rajzoljuk ki! """
        msg = Marker()
        msg.header.frame_id = "base_link" # <--- GLOBÁLIS KERET
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.ns = "global_target"
        msg.id = 0
        msg.type = Marker.SPHERE
        msg.action = Marker.ADD
        
        # Nincs trükk, nincs eltolás, ez a nyers célpont
        msg.pose.position.x = x / 1000.0
        msg.pose.position.y = y / 1000.0
        msg.pose.position.z = z / 1000.0
        msg.pose.orientation.w = 1.0
        msg.scale.x = 0.02
        msg.scale.y = 0.02
        msg.scale.z = 0.02
        msg.color.r = 0.0
        msg.color.g = 1.0
        msg.color.b = 0.0
        msg.color.a = 1.0
        
        self.marker_pub.publish(msg)

    def publish_joints_all(self, joint_map):
        """
        joint_map: Egy dictionary, ami tartalmazza a szögeket minden lábhoz.
        Pl.: {'leg_1': [0, 10, -20], 'leg_2': ...}
        """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Üres listák, amiket feltöltünk
        msg.name = []
        msg.position = []
        
        # Végigmegyünk a 6 lábon (sorrendben, hogy a ROS ne zavarodjon össze)
        # Fontos: A neveknek pontosan egyezniük kell az URDF-fel!
        leg_order = ["leg_1", "leg_2", "leg_3", "leg_4", "leg_5", "leg_6"]
        
        for leg_name in leg_order:
            if leg_name in joint_map:
                coxa, femur, tibia = joint_map[leg_name]
                
                # URDF nevek generálása (pl. joint_1_coxa)
                # A te URDF-edben a lábak számozása 1-től 6-ig megy?
                # Feltételezem: joint_1_coxa, joint_2_coxa, stb.
                idx = leg_name.split('_')[1] # Kiveszi a számot (pl. "1")
                
                msg.name.append(f'joint_{idx}_coxa')
                msg.name.append(f'joint_{idx}_femur')
                msg.name.append(f'joint_{idx}_tibia')
                
                msg.position.append(coxa)
                msg.position.append(femur)
                msg.position.append(tibia)
        
        self.joint_pub.publish(msg)

    def timer_callback(self):
        now = time.time()
        t = now - self.start_time
        speed = 4.0  # Gyorsabb, hogy látszódjon a ritmus
        
        # JÁRÁS PARAMÉTEREI
        step_length = 40.0    # Milyen hosszút lépjen (mm)
        step_height = 30.0    # Milyen magasra emelje a lábát (mm)
        base_dist = 220.0     # Alap terpesz
        base_height = -100.0  # Test magassága a földtől

        # Csoportok definiálása (Tripod)
        # A csoport: 1, 3, 5 (Indexek: 1, 3, 5)
        # B csoport: 2, 4, 6 (Indexek: 2, 4, 6)
        group_a = ["leg_1", "leg_3", "leg_5"]

        all_joints = {}

        for leg_key, leg_cfg in self.LEGS.items():
            
            # 1. FÁZIS KISZÁMÍTÁSA
            # Ha 'A' csoport, akkor 0 eltolás, ha 'B', akkor PI (180 fok)
            phase_offset = 0.0
            if leg_key not in group_a:
                phase_offset = math.pi 

            # A közös időalap + a láb saját fázisa
            current_t = (t * speed) + phase_offset

            # 2. TRAJEKTÓRIA (Pályagörbe) GENERÁLÁSA
            # Ez egy nagyon egyszerű szinuszos járás:
            
            # X irány (Előre-Hátra mozgás a saját síkjában)
            # Amikor sin > 0, előre megy, amikor sin < 0, hátra tol.
            # A radius_offset pozitív és negatív között ingadozik.
            radius_offset = math.cos(current_t) * (step_length / 2.0)
            
            # Z irány (Fel-Le mozgás)
            # Itt a trükk: A lábat csak akkor emeljük, amikor ELŐRE megy (Swing).
            # Amikor hátra tol (Stance), a földön kell maradnia.
            # math.sin(current_t) > 0 esetén emelünk, egyébként 0 (földön van).
            
            raw_sine = math.sin(current_t)
            height_offset = 0.0
            
            if raw_sine > 0:
                # Félkör pálya a levegőben (Swing fázis)
                height_offset = raw_sine * step_height
            else:
                # A földön húzza (Stance fázis) - Itt egyenes vonal
                height_offset = 0.0

            # 3. KOORDINÁTÁK
            angle = leg_cfg["mount_angle_rad"]
            
            # Sugárirányú mozgás (előre-hátra lépkedés)
            current_radius = base_dist + radius_offset
            
            target_x = current_radius * math.cos(angle)
            target_y = current_radius * math.sin(angle)
            target_z = base_height + height_offset

            # 4. MATEK
            lx, ly, lz = self.body_to_leg_coords(target_x, target_y, target_z, leg_cfg)
            coxa, femur, tibia = self.compute_ik(lx, ly, lz)
            all_joints[leg_key] = [coxa, femur, tibia]

            # Marker az 1-es lábhoz (Hogy lásd a félkör pályát!)
            if leg_key == "leg_1":
                self.publish_marker_global(target_x, target_y, target_z)

        self.publish_joints_all(all_joints)

def main(args=None):
    rclpy.init(args=args)
    node = HexapodController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()