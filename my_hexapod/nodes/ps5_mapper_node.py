import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class PS5MapperNode(Node):
    def __init__(self):
        super().__init__('ps5_mapper')
        
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel_joy', 10) 
        self.cmd_pub = self.create_publisher(String, 'robot_command', 10)
        
        # --- ÚJ: D-pad hozzáadva gombként ---
        self.BUTTON_MAP = {
            0: "CROSS",     # STOP / Vészfék
            1: "CIRCLE",    # ANIM_WAVE / AI_GESTURE
            2: "SQUARE",    # ANIM_ATTACK / AI_FOLLOW
            3: "TRIANGLE",  # AI_SENTRY
            6: "OPTIONS",   # Módváltó gomb!
            11: "DPAD_UP",  # Tripod
            12: "DPAD_DOWN",# Ripple
            13: "DPAD_LEFT",# Bi-Gait
            14: "DPAD_RIGHT"# Wave
        }
        
        self.last_buttons = []
        self.current_mode = "MANUAL" 
        
        self.get_logger().info("🎮 PS5 Mapper Node elindult! D-Pad járásmódok aktiválva.")

    def joy_callback(self, msg):
        # 1. GOMBOK FELDOLGOZÁSA
        if not self.last_buttons:
            self.last_buttons = list(msg.buttons)
            return
            
        for i, state in enumerate(msg.buttons):
            if state == 1 and self.last_buttons[i] == 0:
                button_name = self.BUTTON_MAP.get(i, f"UNKNOWN_{i}")
                self.process_button_press(button_name)
                
        self.last_buttons = list(msg.buttons)
        
        # 2. ANALÓG KAROK (Csak MANUAL módban)
        if self.current_mode == "MANUAL":
            twist = Twist()
            twist.linear.x = msg.axes[1]   
            twist.linear.y = msg.axes[0]   
            twist.angular.z = msg.axes[2]  
            
            l2_pull = (1.0 - msg.axes[4]) / 2.0  
            r2_pull = (1.0 - msg.axes[5]) / 2.0  
            twist.linear.z = (l2_pull - r2_pull)
            
            self.vel_pub.publish(twist)

    def process_button_press(self, btn_name):
        cmd_msg = String()
        
        if btn_name == "CROSS":      
            cmd_msg.data = "STOP"
            
        elif btn_name == "OPTIONS":
            self.current_mode = "AI" if self.current_mode == "MANUAL" else "MANUAL"
            cmd_msg.data = f"SYS_MODE_{self.current_mode}"
            self.get_logger().info(f"🔄 Kontroller átváltva: {self.current_mode} módba!")

        elif self.current_mode == "MANUAL":
            # Animációk
            if btn_name == "SQUARE": cmd_msg.data = "ANIM_ATTACK"
            elif btn_name == "CIRCLE": cmd_msg.data = "ANIM_WAVE" 
            # ÚJ: Járásmódok a D-PAD-en
            elif btn_name == "DPAD_UP": cmd_msg.data = "GAIT_TRIPOD"
            elif btn_name == "DPAD_DOWN": cmd_msg.data = "GAIT_RIPPLE"
            elif btn_name == "DPAD_LEFT": cmd_msg.data = "GAIT_BI"
            elif btn_name == "DPAD_RIGHT": cmd_msg.data = "GAIT_WAVE"
            else: return
            
        elif self.current_mode == "AI":
            if btn_name == "SQUARE": cmd_msg.data = "AI_FOLLOW"
            elif btn_name == "CIRCLE": cmd_msg.data = "AI_GESTURE" 
            elif btn_name == "TRIANGLE": cmd_msg.data = "AI_SENTRY"
            else: return
            
        self.cmd_pub.publish(cmd_msg)
        self.get_logger().info(f"📤 Parancs kiküldve: {cmd_msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = PS5MapperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()