import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Empty
import rclpy.client

class HexapodBrainNode(Node):
    def __init__(self):
        super().__init__('hexapod_brain')
        
        # --- 1. ÁLLAPOTVÁLTOZÓK ---
        self.control_mode = "MANUAL"      
        self.current_state = "IDLE"       
        self.active_gait = "TRIPOD"       
        self.active_animation = "NONE"    
        
        # ÚJ: Belső állapot az AI viselkedéshez
        self.active_ai_mode = "STANDBY"
        
        self.joy_vel = Twist()
        self.ai_vel = Twist()
        
        # --- 2. FELIRATKOZÁSOK ---
        self.create_subscription(Twist, 'cmd_vel_joy', self.joy_vel_callback, 10)
        self.create_subscription(Twist, 'cmd_vel_ai', self.ai_vel_callback, 10)
        self.create_subscription(String, 'robot_command', self.command_callback, 10)
        
        # --- 3. PUBLISHEREK ---
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.state_pub = self.create_publisher(String, 'robot_state', 10)
        
        # ÚJ: Ezen a csatornán mondjuk meg az AI Node-nak, hogy mit csináljon a képpel!
        self.ai_mode_pub = self.create_publisher(String, 'ai_current_mode', 10) 
        self.vio_mode_pub = self.create_publisher(String, 'vio_current_mode', 10)
        
        self.rtab_reset_client = self.create_client(Empty, '/rtabmap/reset')
        self.odom_reset_client = self.create_client(Empty, '/odom/reset')
        
        self.timer = self.create_timer(1.0 / 50.0, self.timer_callback)
        self.get_logger().info("Agy 3.0 (AI Menürendszerrel) online! 🧠")

    def joy_vel_callback(self, msg): self.joy_vel = msg
    def ai_vel_callback(self, msg): self.ai_vel = msg

    def command_callback(self, msg):
        cmd = msg.data
        
        # --- 0. HALÁL / KÓMA MÓD ---
        if cmd == "SYS_DEAD":
            self.control_mode = "DEAD"
            self.current_state = "ANIMATION"
            self.active_animation = "SIT"
            self.joy_vel = Twist()
            self.ai_vel = Twist()
            self.get_logger().error("☠️ RENDSZER LEZÁRVA! (Akku kritikus, robot leültetve)")
            return

        # Ha a robot "halott", semmilyen további parancsot nem engedünk át!
        if self.control_mode == "DEAD":
            return
        
        # 1. Rendszerszintű módváltás (Manual <-> AI)
        if cmd.startswith("SYS_MODE_"):
            self.control_mode = cmd.replace("SYS_MODE_", "")
            self.get_logger().info(f"⚙️ Rendszer átállítva: {self.control_mode} vezérlésre")
            
            self.joy_vel = Twist()
            self.ai_vel = Twist()
            
            if self.control_mode == "AI":
                self.active_ai_mode = "STANDBY"
                self.ai_mode_pub.publish(String(data=self.active_ai_mode))
                self.vio_mode_pub.publish(String(data="STANDBY")) # VIO alszik
                self.current_state = "IDLE"
                
            elif self.control_mode == "VIO":
                self.active_ai_mode = "STANDBY"
                self.ai_mode_pub.publish(String(data=self.active_ai_mode)) # AI YOLO kikapcs
                self.vio_mode_pub.publish(String(data="ACTIVE"))         # VIO Depth bekapcs!
                self.current_state = "IDLE"
            
            elif self.control_mode == "MANUAL":
                self.active_ai_mode = "STANDBY"
                self.ai_mode_pub.publish(String(data=self.active_ai_mode)) # AI alszik
                self.vio_mode_pub.publish(String(data="STANDBY")) # VIO alszik
                self.current_state = "IDLE"
                self.active_animation = "NONE"
                
                # ---> ÚJ: Töröljük a memóriát <---
                self.get_logger().info("🧹 VIO kikapcsolva: Térkép és Odometria memóriájának törlése...")
                if self.rtab_reset_client.wait_for_service(timeout_sec=1.0):
                    self.rtab_reset_client.call_async(Empty.Request())
                if self.odom_reset_client.wait_for_service(timeout_sec=1.0):
                    self.odom_reset_client.call_async(Empty.Request())
            
        # 2. Vészmegállás (Kizárólag a PS5 "X" gombja küldi)
        elif cmd == "STOP":
            self.current_state = "IDLE"
            self.active_animation = "NONE"
            
            # --- ÚJ: Fantom sebességek törlése megálláskor! ---
            self.joy_vel = Twist()
            self.ai_vel = Twist()
            
            if self.control_mode == "AI":
                self.active_ai_mode = "STANDBY"
                self.ai_mode_pub.publish(String(data=self.active_ai_mode))
            self.get_logger().warn("🛑 STOP! Visszatérés IDLE/STANDBY módba.")

        # 3. ÚJ: Animáció leállítása (Az AI küldi, ha kimegy az ember a képből)
        elif cmd == "CLEAR_ANIM":
            self.current_state = "IDLE"
            self.active_animation = "NONE"
            # FIGYELEM: Itt NEM nyúlunk a self.active_ai_mode-hoz!
            # Így a kamera tovább figyel, és nem alszik el a chip!
            self.get_logger().info("⏸️ Animáció leállítva, várakozás a célpontra...")
            
        # 4. AI Viselkedések kiválasztása (Csak ha az AI vezet)
        elif cmd.startswith("AI_") and self.control_mode == "AI":
            self.active_ai_mode = cmd.replace("AI_", "") 
            self.ai_mode_pub.publish(String(data=self.active_ai_mode))
            self.get_logger().info(f"🧠 AI Mód aktiválva: {self.active_ai_mode}")
            
        # 5. Kézi Járásmódok (Csak ha MANUAL)
        elif cmd.startswith("GAIT_") and self.control_mode == "MANUAL":
            self.active_gait = cmd.replace("GAIT_", "")
            self.get_logger().info(f"🐾 Járásmód: {self.active_gait}")
            
        # 6. Animációk indítása
        elif cmd.startswith("ANIM_"):
            self.active_animation = cmd.replace("ANIM_", "")
            self.current_state = "ANIMATION" 
            self.get_logger().info(f"🎬 Animáció indítása: {self.active_animation}")

    def timer_callback(self):
        current_input_vel = self.ai_vel if self.control_mode == "AI" else self.joy_vel
        
        if self.current_state != "ANIMATION":
            # Mozgás érzékelése
            if abs(current_input_vel.linear.x) > 0.1 or \
               abs(current_input_vel.linear.y) > 0.1 or \
               abs(current_input_vel.angular.z) > 0.1:
                self.current_state = "WALK"
            else:
                self.current_state = "IDLE"

        state_msg = String()
        if self.current_state == "IDLE":
            state_msg.data = "IDLE"
        elif self.current_state == "WALK":
            state_msg.data = f"WALK_{self.active_gait}"
        elif self.current_state == "ANIMATION":
            state_msg.data = f"ANIM_{self.active_animation}"
            
        self.state_pub.publish(state_msg)
        
        if self.current_state == "ANIMATION" or (self.control_mode == "AI" and self.active_ai_mode == "STANDBY"):
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