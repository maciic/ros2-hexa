import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
import math
import time

class HexapodController(Node):
    def __init__(self):
        super().__init__('hexapod_controller')
        
        # Fizikai méretek
        self.femurLength = 110.0
        self.tibiaLength = 196.0
        self.z_offset = 0.0

        # === ÚJ, REÁLIS CÉLPONTOK (Vállhoz viszonyítva) ===
        # x: előre/hátra, y: balra ki, z: lefelé a földig
        self.p1 = {"x": 130.0, "y": 150.0, "z": 0.0}  # Lépés előre
        self.p2 = {"x": -130.0, "y": 150.0, "z": 0.0} # Lépés hátra
        
        self.step_height = 80.0  # 8 centit emel a lábán
        self.cycle_time = 2.0 

        # ROS csatornák
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.marker_pub = self.create_publisher(Marker, 'target_marker', 10)
        
        self.timer = self.create_timer(1.0 / 50.0, self.timer_callback)
        self.start_time = time.time()

    def computeAngle(self, x, y, z):
        # A te matematikai képleted (Érintetlenül!)
        actual_z = self.z_offset - z
        R = math.sqrt(x**2 + y**2)
        D = math.sqrt(R**2 + actual_z**2)

        D = max(min(D, self.femurLength + self.tibiaLength - 0.1), 0.1)

        fi = math.atan2(y, x)
        gamma = math.degrees(math.atan2(R, actual_z))
        
        val_alpha = (self.femurLength**2 + D**2 - self.tibiaLength**2) / (2 * self.femurLength * D)
        alpha = math.degrees(math.acos(max(min(val_alpha, 1.0), -1.0)))
        
        val_beta = (self.tibiaLength**2 + self.femurLength**2 - D**2) / (2 * self.femurLength * self.tibiaLength)
        beta  = math.acos(max(min(val_beta, 1.0), -1.0))

        fi_deg = 180.0 - round(math.degrees(fi))
        alpha_deg = 180.0 - round((gamma + 20.0) - alpha)
        beta_deg = 180.0 - round(math.degrees(beta))
        
        return fi_deg, alpha_deg, beta_deg

    def calculate_trajectory(self, t):
        cycle = t % self.cycle_time
        phase = cycle / (self.cycle_time / 2.0)
        
        x1, y1, z1 = self.p1["x"], self.p1["y"], self.p1["z"]
        x2, y2, z2 = self.p2["x"], self.p2["y"], self.p2["z"]

        if cycle < self.cycle_time / 2.0:
            # === JAVÍTOTT EMELÉS ===
            t_smooth = (1 - math.cos(phase * math.pi)) / 2 
            x = x2 + (x1 - x2) * t_smooth  
            y = y1 + (y2 - y1) * t_smooth
            z_ground = z1 + (z2 - z1) * t_smooth
            
            z_lift = self.step_height * (1 - (2 * t_smooth - 1)**2)
            z = z_ground + z_lift # <--- ITT A LÉNYEG! PLUSZ JEL, HOGY FELFELÉ EMELJEN
        else:
            phase_stance = phase - 1.0
            x = x1 + (x2 - x1) * phase_stance 
            y = y1 + (y2 - y1) * phase_stance
            z = z1
            
        return x, y, z

    def publish_marker(self, x, y, z):
        msg = Marker()
        # === JAVÍTOTT GÖMB POZÍCIÓ ===
        # A világ fix pontjához (base_link) kötjük, hogy ne forogjon a lábbal!
        msg.header.frame_id = "base_link" 
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.ns = "ik_target"
        msg.id = 0
        msg.type = Marker.SPHERE
        msg.action = Marker.ADD
        
        # Hozzáadjuk a bal első váll URDF koordinátáit (X=0.1, Y=0.06), hogy a helyére kerüljön
        msg.pose.position.x = (x / 1000.0) + 0.1
        msg.pose.position.y = (y / 1000.0) + 0.06
        msg.pose.position.z = z / 1000.0
        msg.pose.orientation.w = 1.0
        
        msg.scale.x = 0.03
        msg.scale.y = 0.03
        msg.scale.z = 0.03
        
        msg.color.r = 0.0
        msg.color.g = 1.0
        msg.color.b = 0.0
        msg.color.a = 1.0
        
        self.marker_pub.publish(msg)

    def publish_joints(self, coxa_deg, femur_deg, tibia_deg):
        # =======================================================
        # 🔄 VIZUÁLIS KORREKCIÓ (Szimulátor szinkronizálása)
        # Ha a láb fordítva hajlik a Foxglove-ban, mint ahogy a 
        # zöld gömb mutatja, itt egyszerűen meg tudod fordítani!
        # A (180.0 - szög) megfordítja a forgásirányt.
        # =======================================================
        
        visual_coxa = coxa_deg               # A csípő jónak tűnik
        visual_femur = 180.0 - femur_deg     # MEGFORDÍTJUK A COMBOT!
        visual_tibia = 180.0 - tibia_deg     # MEGFORDÍTJUK A LÁBSZÁRAT IS! (Ha szükséges)
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint_1_coxa', 'joint_1_femur', 'joint_1_tibia']
        
        msg.position = [
            math.radians(visual_coxa),
            math.radians(visual_femur),
            math.radians(visual_tibia)
        ]
        self.joint_pub.publish(msg)

    def timer_callback(self):
        t = time.time() - self.start_time
        x, y, z = self.calculate_trajectory(t)
        self.publish_marker(x, y, z)
        coxa, femur, tibia = self.computeAngle(x, y, z)
        self.publish_joints(coxa, femur, tibia)

def main(args=None):
    rclpy.init(args=args)
    node = HexapodController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()