import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time

class HexapodBrainNode(Node):
    def __init__(self):
        super().__init__('hexapod_brain')
        
        # --- 1. AZ ÁLLAPOTGÉP VÁLTOZÓI (STATE MACHINE) ---
        self.control_mode = "MANUAL"      # Ki vezet? MANUAL vagy AI
        self.current_state = "IDLE"       # Fő állapot: IDLE, WALK, ANIMATION
        self.active_gait = "TRIPOD"       # Al-állapot sétához: TRIPOD, RIPPLE
        self.active_animation = "NONE"    # Al-állapot animációhoz: ATTACK, WAVE, NONE
        
        # Bemeneti sebességek tárolása
        self.joy_vel = Twist()
        self.ai_vel = Twist()
        
        # --- 2. FELIRATKOZÁSOK (Bemenetek) ---
        self.create_subscription(Twist, 'cmd_vel_joy', self.joy_vel_callback, 10)
        self.create_subscription(Twist, 'cmd_vel_ai', self.ai_vel_callback, 10)
        self.create_subscription(String, 'robot_command', self.command_callback, 10)
        
        # --- 3. PUBLISHEREK (Kimenetek az Izomzat felé) ---
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.state_pub = self.create_publisher(String, 'robot_state', 10) # <--- ÚJ: A hivatalos állapot
        
        # 50 Hz-es időzítő a logikához
        self.timer = self.create_timer(1.0 / 50.0, self.timer_callback)
        
        self.get_logger().info("Agy 2.0 (Állapotgép) online! 🧠")

    def joy_vel_callback(self, msg):
        self.joy_vel = msg

    def ai_vel_callback(self, msg):
        self.ai_vel = msg

    def command_callback(self, msg):
        """ Itt dolgozzuk fel a Mapperből jövő gombnyomásokat """
        cmd = msg.data
        
        # 1. Vezérlési mód váltása
        if cmd == "TOGGLE_AI":
            self.control_mode = "AI" if self.control_mode == "MANUAL" else "MANUAL"
            self.get_logger().info(f"Vezérlés átadva: {self.control_mode}")
            
        # 2. Vészmegállás / Reset
        elif cmd == "STOP":
            self.current_state = "IDLE"
            self.active_animation = "NONE"
            self.get_logger().warn("STOP! Visszatérés IDLE módba.")
            
        # 3. Járásmód (Gait) beállítása (Pl. GAIT_RIPPLE)
        elif cmd.startswith("GAIT_"):
            self.active_gait = cmd.replace("GAIT_", "")
            self.get_logger().info(f"Kiválasztott járásmód: {self.active_gait}")
            
        # 4. Animációk indítása (Pl. ANIM_ATTACK)
        elif cmd.startswith("ANIM_"):
            self.active_animation = cmd.replace("ANIM_", "")
            self.current_state = "ANIMATION" # Azonnal felülírjuk az állapotot!
            self.get_logger().info(f"Animáció indítása: {self.active_animation}")

    def timer_callback(self):
        """ Ez fut le másodpercenként 50-szer. Itt dől el a végső állapot! """
        
        # 1. Melyik sebességet figyeljük?
        current_input_vel = self.joy_vel if self.control_mode == "MANUAL" else self.ai_vel
        
        # 2. ÁLLAPOT DÖNTÉS
        # Ha épp animációt játszunk, nem engedjük a sétát. Csak a STOP gomb szakíthatja meg.
        if self.current_state != "ANIMATION":
            # Van mozgás parancs a karokon? (Deadzone ellenőrzés)
            if abs(current_input_vel.linear.x) > 0.1 or \
               abs(current_input_vel.linear.y) > 0.1 or \
               abs(current_input_vel.angular.z) > 0.1:
                self.current_state = "WALK"
            else:
                self.current_state = "IDLE"

        # 3. HIVATALOS ÁLLAPOT ÖSSZEÁLLÍTÁSA ÉS KIKÜLDÉSE
        state_msg = String()
        
        if self.current_state == "IDLE":
            state_msg.data = "IDLE"
        elif self.current_state == "WALK":
            state_msg.data = f"WALK_{self.active_gait}" # pl: "WALK_TRIPOD" vagy "WALK_RIPPLE"
        elif self.current_state == "ANIMATION":
            state_msg.data = f"ANIM_{self.active_animation}" # pl: "ANIM_ATTACK"
            
        # 4. ÜZENETEK KÜLDÉSE AZ IZOMZATNAK
        self.state_pub.publish(state_msg)
        
        # Ha animáció fut, lenullázzuk a sebességet, hogy a lábak ne próbáljanak sétálni közben
        if self.current_state == "ANIMATION":
            self.vel_pub.publish(Twist()) 
        else:
            self.vel_pub.publish(current_input_vel)

def main(args=None):
    rclpy.init(args=args)
    node = HexapodBrainNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()