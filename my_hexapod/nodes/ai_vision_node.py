import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class AIVisionNode(Node):
    def __init__(self):
        super().__init__('ai_vision_node')
        
        # 1. PUBLISHER a Főnök felé
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_ai', 10)
        
        # 2. FELIRATKOZÁS A KAMERÁRA (Queue size 1-re csökkentve!)
        self.subscription = self.create_subscription(Image, 'image_raw', self.image_callback, 1)
        self.bridge = CvBridge() 
        
        # ÚJ: Egy flag, hogy épp dolgozunk-e
        self.is_processing = False
        
        # 3. IDEIGLENES FELISMERŐ MOTOR
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        
        # 4. SZABÁLYOZÓ MATEK
        self.center_x = 320 
        self.target_width = 150 
        self.kp_yaw = 0.002  
        self.kp_linear = 0.005 
        
        self.get_logger().info("👁️ AI Látókéreg online! Várom a képeket a kamerától...")

    def image_callback(self, msg):
        # --- 1. Frame eldobása, ha az AI épp egy korábbi képen izzad ---
        if getattr(self, 'is_processing', False):
            return
            
        # Zároljuk a bemenetet: mostantól mi dolgozunk!
        self.is_processing = True

        try:
            # A TELJES KORÁBBI LOGIKA ITT VAN BENT (egy tabbal beljebb húzva)
            
            # Kép átalakítása
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            frame = cv2.resize(frame, (640, 480))

            # Ember keresése
            boxes, weights = self.hog.detectMultiScale(frame, winStride=(8,8), padding=(8, 8), scale=1.05)
            
            ai_cmd = Twist()

            if len(boxes) > 0:
                x, y, w, h = boxes[0]
                box_center_x = x + (w / 2)
                
                error_yaw = self.center_x - box_center_x 
                ai_cmd.angular.z = error_yaw * self.kp_yaw
                
                error_linear = self.target_width - w
                ai_cmd.linear.x = error_linear * self.kp_linear
                
                ai_cmd.angular.z = max(min(ai_cmd.angular.z, 0.5), -0.5)
                ai_cmd.linear.x = max(min(ai_cmd.linear.x, 0.5), -0.5)

                irany = "BALRA ⬅️" if ai_cmd.angular.z > 0 else "JOBBRA ➡️" if ai_cmd.angular.z < 0 else "KÖZÉPEN 🎯"
                tav = "ELŐRE ⬆️" if ai_cmd.linear.x > 0 else "HÁTRA ⬇️" if ai_cmd.linear.x < 0 else "ÁLL 🛑"
                
                self.get_logger().info(
                    f"LÁTOK VALAKIT! [Közép: {box_center_x:.0f}, Szélesség: {w}] -> "
                    f"Forgás: {irany}, Séta: {tav}"
                )
            else:
                self.get_logger().info("Senki nincs a képen... 😴")

            # Parancs kiküldése az Agynak
            self.cmd_pub.publish(ai_cmd)

        except Exception as e:
            # Ha bármi elszállna feldolgozás közben, itt elkapjuk
            self.get_logger().error(f"Hiba a képfeldolgozás során: {e}")
            
        finally:
            # --- 2. ZÁROLÁS FELOLDÁSA ---
            # Ez a rész MINDIG lefut, akár volt hiba, akár sikeres volt a felismerés.
            # Jöhet a következő legfrissebb képkocka!
            self.is_processing = False

def main(args=None):
    rclpy.init(args=args)
    node = AIVisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()