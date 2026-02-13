import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time

class LegControllerNode(Node):
    def __init__(self):
        super().__init__('leg_controller_node')
        
        # Létrehozunk egy "kiadót" (publisher), ami a /joint_states csatornára küld
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        
        # Időzítő: Másodpercenként 30-szor (30 Hz) lefut a timer_callback függvény
        timer_period = 1.0 / 30.0  
        self.timer = self.create_timer(timer_period, self.autoMove)
        self.start_time = time.time()

    def calculate_ik(self, x, y, z):

        femurLength = 110
        tibiaLength = 196
        Z_OFFSET = 0  # z eltolás a váll síkhoz képest

        actual_z = Z_OFFSET - z 

        R = math.sqrt(x**2 + y**2)
        D = math.sqrt(R**2 + actual_z**2)
        D = max(10, min(D, femurLength + tibiaLength - 1))

        fi = math.atan2(y, x)
        gamma = math.degrees(math.atan2(R, actual_z))
        cos_a = (femurLength**2 + D**2 - tibiaLength**2) / (2 * femurLength * D)

        alpha_internal = math.degrees(math.acos(max(-1, min(1, cos_a))))
        
        # beta_internal: A térd belső szöge
        cos_b = (femurLength**2 + tibiaLength**2 - D**2) / (2 * femurLength * tibiaLength)
        beta_internal = math.degrees(math.acos(max(-1, min(1, cos_b))))
        
        coxa_angle = 180 - round(math.degrees(fi))
        femur_angle = 180 - round((gamma + 20) - alpha_internal)
        tibia_angle = 180 - round(beta_internal)

        coxa_angle = math.radians(coxa_angle)
        femur_angle = math.radians(femur_angle)
        tibia_angle = math.radians(tibia_angle)

        return coxa_angle, femur_angle, tibia_angle

    def timer_callback(self):
        # 1. Cél koordináták kitalálása (pl. rajzoljunk egy kört a levegőben)
        # Később ezt egy joystickról vagy másik programból is kaphatja!
        t = time.time() - self.start_time
        target_x = 0.1 + math.sin(t) * 0.05
        target_y = 0.1
        target_z = -0.1 + math.cos(t) * 0.05

        # 2. Szögek kiszámolása a te kódoddal
        coxa, femur, tibia = self.calculate_ik(target_x, target_y, target_z)

        # 3. Az üzenet (JointState) összeállítása és elküldése
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # A neveknek PONTOSAN egyezniük kell az URDF-ben megadott nevekkel!
        msg.name = ['joint_1_coxa', 'joint_1_femur', 'joint_1_tibia']
        msg.position = [coxa, femur, tibia]
        
        self.publisher_.publish(msg)

    # === STEP-GAIT TRAJECTORY (parabolikus lábemelés) ===
    def step_gait(self, p1, p2, steps=120, step_height=50, delay=0.005):
        x1, y1, z1 = p1["x"], p1["y"], p1["z"]
        x2, y2, z2 = p2["x"], p2["y"], p2["z"]

        for i in range(steps + 1):
            # S-görbe (Cos alapú) simítás a darabosság ellen
            t_linear = i / steps
            t_smooth = (1 - math.cos(t_linear * math.pi)) / 2 

            # X-Y mozgás a simított t értékkel
            x = x1 + (x2 - x1) * t_smooth
            y = y1 + (y2 - y1) * t_smooth
            
            # Z emelés (marad a parabola, de a simított t-vel)
            z_ground = z1 + (z2 - z1) * t_smooth
            z_lift = step_height * (1 - (2 * t_smooth - 1)**2)
            z = z_ground - z_lift

            fi, alpha, beta = self.calculate_ik(x, y, z)
            #move_servos(fi, alpha, beta)
            
            # Minimális delay a szervó reakcióideje miatt
            time.sleep(delay)

    def stance_gait(self, p1, p2, steps=60, delay=0.02):
        """A láb a földön van, tolja a testet."""
        x1, y1, z1 = p1["x"], p1["y"], p1["z"]
        x2, y2 = p2["x"], p2["y"]

        for i in range(steps + 1):
            t = i / steps
            x = x1 + (x2 - x1) * t
            y = y1 + (y2 - y1) * t
            fi, alpha, beta = self.calculate_ik(x, y, z1)
            #move_servos(fi, alpha, beta)
            time.sleep(delay)


    def autoMove(self):
        TEST_POINTS = [
            {"x": 150, "y": 140, "z": 0, "leiras": "Start"},
            {"x": -150, "y": 140, "z": 0, "leiras": "Előre"},
            {"x": 150, "y": 140, "z": 0, "leiras": "Vissza"},
        ]

        for i in range(3):
            p1 = TEST_POINTS[0]
            print(p1)
            p2 = TEST_POINTS[1]
            p3 = TEST_POINTS[2]
            print(f"➡️ {p1['leiras']} → {p2['leiras']}")
            self.step_gait(p1, p2)
            time.sleep(2)
            print(f"➡️ {p2['leiras']} → {p3['leiras']}")
            self.stance_gait(p2, p3, steps=55, delay=0.012)
    
def main(args=None):
    rclpy.init(args=args)
    node = LegControllerNode()
    rclpy.spin(node) # Ez tartja életben a programot (fut a végtelenségig)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()