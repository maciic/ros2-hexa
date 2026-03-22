import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist 
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String
import json
import time
import math
import os

# === IMPORTÁLJUK A MODULOKAT ===
from my_hexapod.core.robot_kinematics import HexapodKinematics 
from my_hexapod.core.robot_gait import HexapodGait 
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

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
        self.cmd_vel = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
        self.current_vel = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
        self.ramp_step = 0.02  #Gyorsulás lépésenként (0.01 = 1% gyorsulás minden ciklusban)
        
        self.body_rpy = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.breathe_z = 0.0
        self.anim_leg_offsets = {}
        
        # --- ÚJ: A Főnöktől kapott hivatalos állapot ---
        self.robot_state_str = "IDLE" 
        
        # 4. DINAMIKUS PARAMÉTEREK DEKLARÁLÁSA
        self.declare_parameters(
            namespace='',
            parameters=[
                ('gait_freq', 1.4),
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
        
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # --- ÚJ: Már nem az animációkat, hanem a tiszta robot_state-et figyeljük az Agytól! ---
        self.state_sub = self.create_subscription(String, 'robot_state', self.state_callback, 10) 
        
        # Célpont publikáló a Foxglove-hoz (az 1-es lábhoz)
        self.leg_target_pub = self.create_publisher(PointStamped, 'debug/leg_1_target', 10)
        
        # ÚJ: Lábvégek (mancsok) publikálása a Foxglove 3D-hez
        self.foot_tips_pub = self.create_publisher(Marker, 'debug/foot_tips', 10)
        
        self.timer = self.create_timer(1.0 / 50.0, self.timer_callback)
        self.start_time = time.time()
        
        self.get_logger().info("Hexapod Controller: Buta Izomzat üzemmód (Végrehajtó) aktív! 💪")

    def cmd_vel_callback(self, msg):
        self.cmd_vel['x'] = msg.linear.x
        self.cmd_vel['y'] = msg.linear.y
        self.cmd_vel['z'] = msg.linear.z  # <--- ÚJ
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
        # ÚJ: A get_tripod_phase-t lecseréltük az általános get_leg_phase-re!
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

        # --- ÚJ: Lekérjük az esetleges egyéni láb animációt (pl. integetés) ---
        anim_off = self.anim_leg_offsets.get(leg_key, {'x': 0.0, 'y': 0.0, 'z': 0.0})

        # Végső CÉLPONT (Test koordinátarendszerben)
        # Ez az a pont a térben, ahova a láb végét tenni akarjuk (Body IK nélkül)
        # ÚJ: Hozzáadjuk az anim_off értékeket az X és Y tengelyhez is
        target_x = neutral_x + off_walk + dx_rot + anim_off['x']
        target_y = neutral_y + off_strafe + dy_rot + anim_off['y']
        
        # Sima, időalapú Z magasság a szinuszgörbéből és a lélegzésből
        # ÚJ: Hozzáadjuk az anim_off['z'] értéket is az esetleges lábemeléshez
        target_z = self.gait.params['base_height'] + off_z + self.breathe_z + anim_off['z']

        # C. KINEMATIKA (Kinematics Module)
        # 1. Transzformáljuk a pontot a láb lokális rendszerébe + Alkalmazzuk a Body IK-t
        lx, ly, lz, global_pos = self.kinematics.body_to_leg_coords(
            target_x, target_y, target_z, leg_cfg, self.body_rpy
        )

        # 2. Kiszámoljuk a szögeket
        # 2. Kiszámoljuk a szögeket
        angles = self.kinematics.compute_ik(lx, ly, lz)
        return angles, global_pos

    def timer_callback(self):
        # 1. Ramping (A szervók mechanikai kímélése)
        self.current_vel['x'] = self.ramp_value(self.current_vel['x'], self.cmd_vel['x'], self.ramp_step)
        self.current_vel['y'] = self.ramp_value(self.current_vel['y'], self.cmd_vel['y'], self.ramp_step)
        self.current_vel['z'] = self.ramp_value(self.current_vel['z'], self.cmd_vel['z'], self.ramp_step)
        self.current_vel['yaw'] = self.ramp_value(self.current_vel['yaw'], self.cmd_vel['yaw'], self.ramp_step)

        # 2. Debug Pub
        d_msg = Twist()
        d_msg.linear.x = self.current_vel['x']
        d_msg.linear.y = self.current_vel['y']
        d_msg.angular.z = self.current_vel['yaw']
        self.debug_pub.publish(d_msg)

        # 3. ENGEDELMESKEDÉS A FŐNÖKNEK (Nincs több találgatás a sebességből!)
        now = time.time()
        t = now - self.start_time
        
        base_h = self.get_parameter('gait_base_height').value
        self.gait.params['base_height'] = base_h + self.current_vel['z']
        
        # Szétszedjük a parancsot (pl: "WALK_RIPPLE" -> main: "WALK", sub: "RIPPLE")
        state_parts = self.robot_state_str.split('_', 1)
        main_state = state_parts[0] 
        sub_state = state_parts[1] if len(state_parts) > 1 else None

        # Továbbítjuk a parancsot a Koreográfusnak / Matematikusnak
        if main_state == "WALK":
            self.gait.current_state = "WALK"
            self.gait.gait_mode = sub_state if sub_state else "TRIPOD"
            self.gait.active_animation = None
        elif main_state == "ANIM":
            self.gait.current_state = "ANIMATION"
            self.gait.active_animation = sub_state
        else: # IDLE parancs érkezett az Agytól
            
            # ÚJ: Ellenőrizzük, hogy a robot fizikailag megállt-e már!
            is_moving = (abs(self.current_vel['x']) > 0.1 or 
                         abs(self.current_vel['y']) > 0.1 or 
                         abs(self.current_vel['yaw']) > 0.1)
            
            if is_moving:
                # Bár az Agy IDLE-t kért, a test még lendületben van (lassul).
                # Maradunk WALK módban, hogy a lábak szépen befejezzék a lépést!
                self.gait.current_state = "WALK"
                # A self.gait.gait_mode-ot békén hagyjuk, így az előző járásmód folyik tovább
            else:
                # Most már tényleg megállt a sebesség. Jöhet a tényleges IDLE (pl. lélegzés).
                self.gait.current_state = "IDLE"
                self.gait.active_animation = None
            
        # Lekérjük a test dőlését és magasságát az aktuális állapot szerint
        self.body_rpy, self.breathe_z, self.anim_leg_offsets = self.gait.get_body_pose(t)

        # 4. Marker üzenet előkészítése (a lábvégeknek)
        marker_msg = Marker()
        marker_msg.header.stamp = self.get_clock().now().to_msg()
        marker_msg.header.frame_id = "base_link"  # Robot központi frame
        marker_msg.ns = "foot_tips"
        marker_msg.id = 0
        marker_msg.type = Marker.SPHERE_LIST # Egy lista gömbökből (pöttyökből)
        marker_msg.action = Marker.ADD
        marker_msg.pose.orientation.w = 1.0 # Alap orientation

        # Pöttyök mérete (mm helyett méterben: 1.5 cm átmérőjű gömbök)
        marker_msg.scale.x = 0.015
        marker_msg.scale.y = 0.015
        marker_msg.scale.z = 0.015

        # Pöttyök alap színe (például élénk zöld, teljes átlátszatlansággal)
        default_color = ColorRGBA()
        default_color.r = 0.0
        default_color.g = 1.0
        default_color.b = 0.0
        default_color.a = 1.0

        # --- Lábak feldolgozása és koordináták begyűjtése ---
        all_joints = {}
        for leg_key, leg_cfg in self.LEGS.items():
            # Most már a process_leg visszaadja az angles mellett a global_pos-t is (az előző lépés miatt)
            angles, global_pos = self.process_leg(leg_key, leg_cfg, t)
            all_joints[leg_key] = angles

            # Pont hozzáadása a Marker listához (mm -> méter konverzióval)
            p = Point()
            p.x = global_pos[0] / 1000.0
            p.y = global_pos[1] / 1000.0
            p.z = global_pos[2] / 1000.0
            marker_msg.points.append(p)

            # Szín hozzáadása (mindegyik láb ugyanazt a zöld színt kapja)
            marker_msg.colors.append(default_color)

        # 5. Publikálás
        self.foot_tips_pub.publish(marker_msg)
        self.publish_joints_all(all_joints)

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