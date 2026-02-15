import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
import math
import time

class HexapodController(Node):
    def __init__(self):
        super().__init__('hexapod_controller')
        
        # 1. FIZIKAI MÉRETEK
        self.coxaLength = 46.4
        self.femurZOffset = -15.0
        
        self.femur_x = 110.0      # Vízszintes hossz
        self.femur_drop = 0.0     # Ha van ejtés (mm)
        
        self.femurLength_eff = math.sqrt(self.femur_x**2 + self.femur_drop**2) # Effektív hossz a görbület miatt
        self.femur_angle_offset = math.atan2(self.femur_drop, self.femur_x)    # A görbület miatt a Femur "beépített" szöge

        self.tibia_x = 184.2      # Példa: Vízszintes távolság
        self.tibia_drop = -52.4    # Példa: Mennyivel van lejjebb a vége? (A görbület miatt)
        
        self.tibiaLength_eff = math.sqrt(self.tibia_x**2 + self.tibia_drop**2) # Effektív hossz a görbület miatt
        self.tibia_angle_offset = math.atan2(self.tibia_drop, self.tibia_x)    # A görbület miatt a Tibia "beépített" szöge

        self.z_offset = 0.0

        # 2. A ROBOT GEOMETRIÁJA (Konfiguráció mm-ben és radiánban)
        # ⚠️ FONTOS: Mostantól RADIÁNT használunk a pontosság miatt!
        
        self.LEGS = {
            "leg_1": {
                "name": "leg_1",
                "mount_x": 84.5,   
                "mount_y": 66.8,  
                "mount_angle_rad": 0.668 
            }
        }

        # === 3. ROS CSATORNÁK ===
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.marker_pub = self.create_publisher(Marker, 'target_marker', 10)
        
        # Frissítési ráta
        self.timer = self.create_timer(1.0 / 50.0, self.timer_callback)
        self.start_time = time.time()
        self.get_logger().info("Hexapod Controller Started (Body-Centric Mode)")

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
        actual_z = (self.z_offset - z) - self.femurZOffset

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
        alpha_rad = (gamma - alpha) + self.femur_angle_offset
        beta_rad = (math.pi - beta) + self.tibia_angle_offset

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

    def publish_joints(self, coxa, femur, tibia):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint_1_coxa', 'joint_1_femur', 'joint_1_tibia']
        
        # Átváltás radiánba a ROS-nak
        # ITT MÉG MINDIG LEHET, HOGY TÜKRÖZNÖD KELL (pl. 180 - femur)
        # Ezt a Foxglove-ban látod majd.

        msg.position = [coxa, femur, tibia]
        self.joint_pub.publish(msg)

    def timer_callback(self):
        # 1. IDŐZÍTÉS (Hogy mozogjon)
        now = time.time()
        t = now - self.start_time
        speed = 1.0  # Sebesség (rad/s) - Állítsd nagyobbra, ha gyorsabban akarod

        # 2. A MOZGÁS PARAMÉTEREI (Test-központú koordinátákban)
        # Hol legyen a kör középpontja?
        base_distance = 300.0  # mm (Távolság a test közepétől síkban)
        base_height = 0.0   # mm (Magasság - a "levegőben" körözzön)
        
        # Mekkora legyen a kör?
        radius_length = 30.0   # mm (Előre-hátra mozgás sugara)
        radius_height = 60.0   # mm (Fel-le mozgás sugara)

        # 3. TRAJEKTÓRIA SZÁMÍTÁS (Szinusz/Koszinusz)
        # Kiszámoljuk az eltolást az idő függvényében
        offset_length = math.cos(t * speed) * radius_length
        offset_height = math.sin(t * speed) * radius_height

        # 4. KOORDINÁTÁK GENERÁLÁSA (A 45 fokos lábhoz!)
        # Mivel a leg_1 45 fokban áll (0.785 rad), az "előre-hátra" mozgást
        # fel kell bontanunk X és Y komponensre.
        
        angle_rad = 0.785 # 45 fok
        
        # Az aktuális távolság a test középpontjától
        current_dist = base_distance + offset_length
        
        target_x = current_dist * math.cos(angle_rad)
        target_y = current_dist * math.sin(angle_rad)
        target_z = base_height + offset_height

        # 5. VIZUALIZÁCIÓ (Zöld gömb mozgatása)
        self.publish_marker_global(target_x, target_y, target_z)

        # 6. MATEK & MOZGATÁS (Ez a rész változatlan!)
        # Átalakítjuk lokális koordinátákra
        lx, ly, lz = self.body_to_leg_coords(target_x, target_y, target_z, self.LEGS["leg_1"])
        
        # IK kiszámolása
        coxa_deg, femur_deg, tibia_deg = self.compute_ik(lx, ly, lz)
        
        # Logolás (hogy lásd a számokat pörögni)
        # self.get_logger().info(f'Z: {target_z:.1f} -> Femur: {femur_deg:.1f}')

        # Küldés a robotnak
        self.publish_joints(coxa_deg, femur_deg, tibia_deg)

def main(args=None):
    rclpy.init(args=args)
    node = HexapodController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()