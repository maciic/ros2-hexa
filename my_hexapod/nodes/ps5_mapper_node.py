import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class PS5MapperNode(Node):
    def __init__(self):
        super().__init__('ps5_mapper')
        
        # 1. Feliratkozunk a nyers PS5 adatokra
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        
        # 2. Létrehozzuk a kimeneteket a Főnök (Brain) felé
        # A mozgás parancsok (később a Mux fogja ezeket szűrni)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel_joy', 10) 
        # Magas szintű parancsok (animációk, módváltás)
        self.cmd_pub = self.create_publisher(String, 'robot_command', 10)
        
        # 3. GOMBKIOSZTÁS SZÓTÁR (Ezt bármikor könnyen átírhatod!)
        # Megjegyzés: A Linux joy driver indexei picit eltérhetnek, teszteléssel majd pontosítjuk!
        self.BUTTON_MAP = {
            0: "CROSS",     # X (Stop)
            1: "CIRCLE",    # Kör (Integetés)
            2: "TRIANGLE",  
            3: "SQUARE",    # Négyzet (Támadás)
            4: "L1",        # <--- EZT FELEJTETTEM EL! (Tripod)
            5: "R1",        # <--- ÉS EZT IS! (Ripple)
            9: "OPTIONS",   # AI
            13: "DPAD_UP",  
            14: "DPAD_DOWN" 
        }
        
        # Változó az "Edge Detection"-höz (előző állapot tárolása)
        self.last_buttons = []
        
        self.get_logger().info("PS5 Mapper Node elindult! Várja a parancsokat 🎮")

    def joy_callback(self, msg):
        # --- 1. GOMBOK FELDOLGOZÁSA (Kattintás érzékelése) ---
        if not self.last_buttons:
            self.last_buttons = list(msg.buttons)
            return
            
        for i, state in enumerate(msg.buttons):
            # Ha most le van nyomva (1), de az előző pillanatban nem volt (0)
            if state == 1 and self.last_buttons[i] == 0:
                button_name = self.BUTTON_MAP.get(i, f"UNKNOWN_{i}")
                self.process_button_press(button_name)
                
        self.last_buttons = list(msg.buttons)
        
        # --- 2. ANALÓG KAROK (Séta és Forgás) ---
        twist = Twist()
        # Általában axes[1] a bal fel/le, axes[0] a bal jobbra/balra, axes[3] a jobb jobbra/balra
        twist.linear.x = msg.axes[1]   # Előre/Hátra
        twist.linear.y = msg.axes[0]   # Oldalazás
        twist.angular.z = msg.axes[3]  # Forgás (Yaw)
        
        # --- 3. EXTRA: TESTMAGASSÁG A RAVASZOKKAL (L2 / R2) ---
        # A ravaszok (axes 2 és 5) értéke 1.0 (kiengedve) és -1.0 (behúzva) között mozog.
        # Konvertáljuk át 0.0 és 1.0 közötti értékre:
        l2_pull = (1.0 - msg.axes[2]) / 2.0  
        r2_pull = (1.0 - msg.axes[5]) / 2.0  
        
        # A Z tengely (magasság) eltolását a Twist üzenet linear.z mezőjébe tesszük bele.
        # Ha mindkettőt húzod, az érték 0. Ha R2-t húzod (guggol), negatív, L2-vel pozitív.
        # A Z értéket beszorozzuk mondjuk 40 mm-rel, így ez lesz a maximális emelés/süllyesztés.
        twist.linear.z = (l2_pull - r2_pull) * 40.0 
        
        self.vel_pub.publish(twist)

    def process_button_press(self, btn_name):
        """ Itt határozzuk meg, melyik gomb milyen magas szintű parancsot küldjön """
        cmd_msg = String()
        
        if btn_name == "SQUARE":
            cmd_msg.data = "ANIM_ATTACK"
        elif btn_name == "CIRCLE":
            cmd_msg.data = "ANIM_WAVE" 
        elif btn_name == "CROSS":      # <--- EZ HIÁNYZOTT (STOP / Vészfék)
            cmd_msg.data = "STOP"
        elif btn_name == "L1":         # <--- D-PAD HELYETT: Tripod járás
            cmd_msg.data = "GAIT_TRIPOD"
        elif btn_name == "R1":         # <--- D-PAD HELYETT: Hullámzó (Ripple) járás
            cmd_msg.data = "GAIT_RIPPLE"
        elif btn_name == "OPTIONS":
            cmd_msg.data = "TOGGLE_AI"
        else:
            return 
            
        self.cmd_pub.publish(cmd_msg)
        self.get_logger().info(f"Parancs kiküldve az Agynak: {cmd_msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = PS5MapperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()