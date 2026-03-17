import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2

class AIVisionNode(Node):
    def __init__(self):
        super().__init__('ai_vision_node')
        
        # 1. PUBLISHER a Főnök felé
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_ai', 10)
        
        # 2. KAMERA INDÍTÁSA
        self.cap = cv2.VideoCapture(0) # A /dev/video0 kamera megnyitása
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # 3. IDEIGLENES FELISMERŐ MOTOR (Amíg a Hailo modellt be nem kötjük)
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        
        # 4. SZABÁLYOZÓ MATEK (PID) BEÁLLÍTÁSAI
        self.center_x = 320 # A 640-es képernyő közepe
        self.target_width = 150 # Milyen "széles" legyen az ember (Távolságtartás)
        
        self.kp_yaw = 0.002  # Forgás érzékenysége
        self.kp_linear = 0.005 # Előre-hátra séta érzékenysége
        
        # 5. CIKLUS INDÍTÁSA (10 Hz, hogy a CPU bírja az ideiglenes tesztet)
        self.timer = self.create_timer(0.1, self.process_frame)
        
        self.get_logger().info("👁️ AI Látókéreg online! Keresem a gazdát...")

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Nem kapok képet a kamerától!")
            return

        # Ember keresése a képen (Később itt fogjuk meghívni a Hailo-t!)
        # A detectMultiScale visszaadja a dobozokat: (x, y, szélesség, magasság)
        boxes, weights = self.hog.detectMultiScale(frame, winStride=(8,8), padding=(8, 8), scale=1.05)
        
        ai_cmd = Twist()

        if len(boxes) > 0:
            # Ha látunk embert, kiválasztjuk az elsőt (a legnagyobbat)
            x, y, w, h = boxes[0]
            
            # Doboz közepének kiszámítása
            box_center_x = x + (w / 2)
            
            # --- 1. FORGÁSI LOGIKA (YAW) ---
            # Mennyire van messze a doboz közepe a képernyő közepétől?
            error_yaw = self.center_x - box_center_x 
            ai_cmd.angular.z = error_yaw * self.kp_yaw
            
            # --- 2. TÁVOLSÁG LOGIKA (LINEAR X) ---
            # Mennyire széles az ember a célhoz képest?
            error_linear = self.target_width - w
            ai_cmd.linear.x = error_linear * self.kp_linear
            
            # Limitáljuk a maximális sebességeket, nehogy elszálljon a robot
            ai_cmd.angular.z = max(min(ai_cmd.angular.z, 0.5), -0.5)
            ai_cmd.linear.x = max(min(ai_cmd.linear.x, 0.5), -0.5)

            # Terminálos Vizuális Teszt (Mivel nincs monitorunk)
            irany = "BALRA ⬅️" if ai_cmd.angular.z > 0 else "JOBBRA ➡️" if ai_cmd.angular.z < 0 else "KÖZÉPEN 🎯"
            tav = "ELŐRE ⬆️" if ai_cmd.linear.x > 0 else "HÁTRA ⬇️" if ai_cmd.linear.x < 0 else "ÁLL 🛑"
            
            self.get_logger().info(
                f"LÁTOK VALAKIT! [Közép: {box_center_x:.0f}, Szélesség: {w}] -> "
                f"Forgás: {irany} ({ai_cmd.angular.z:.2f}), Séta: {tav} ({ai_cmd.linear.x:.2f})"
            )
        else:
            # Ha nem látunk senkit, a robot megáll (vagy körbeforoghatna keresve)
            self.get_logger().info("Senki nincs a képen... 😴")

        # Parancs kiküldése az Agynak
        self.cmd_pub.publish(ai_cmd)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AIVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()