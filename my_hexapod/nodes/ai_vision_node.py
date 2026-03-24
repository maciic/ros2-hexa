import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from contextlib import ExitStack

# --- ÚJ: HAILO IMPORTOK ---
from hailo_platform import (HEF, VDevice, HailoStreamInterface, ConfigureParams,
                            InputVStreamParams, OutputVStreamParams, FormatType, InferVStreams)

class AIVisionNode(Node):
    def __init__(self):
        super().__init__('ai_vision_node')
        
        # 1. PUBLISHER a Főnök felé
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_ai', 10)
        
        # 2. FELIRATKOZÁS A KAMERÁRA
        # Itt maradhat a sima image_raw, mert a konténeren belül vagyunk, nem kell a hálózaton átmennie!
        self.subscription = self.create_subscription(Image, 'image_raw', self.image_callback, 1)
        self.bridge = CvBridge() 
        self.is_processing = False 
        
        # 3. SZABÁLYOZÓ MATEK (A paramétereket a YOLO 640x640-es felbontásához igazítottuk)
        self.center_x = 320 
        self.target_width = 250 # Kicsit nagyobbra vettem, mert a YOLO pontosabb dobozt rajzol
        self.kp_yaw = 0.002  
        self.kp_linear = 0.005 

        # --- 4. HAILO AI CHIP INICIALIZÁLÁSA ---
        self.get_logger().info("🧠 Hailo-8 AI inicializálása folyamatban...")
        self.stack = ExitStack()
        
        try:
            # A Docker konténerben a fájl pontos helye:
            hef_path = "src/ros2-hexa/my_hexapod/yolov8s.hef"
            self.hef = HEF(hef_path)
            
            # Csatlakozás a fizikai chiphez
            self.target = self.stack.enter_context(VDevice())
            
            # Modell konfigurálása a chipen
            self.configure_params = ConfigureParams.create_from_hef(hef=self.hef, interface=HailoStreamInterface.PCIe)
            self.network_groups = self.target.configure(self.hef, self.configure_params)
            self.network_group = self.network_groups[0]
            self.network_group_params = self.network_group.create_params()
            
            # Bemeneti és kimeneti adatcsatornák (VStreams) létrehozása
            self.input_vstreams_params = InputVStreamParams.make(self.network_group, format_type=FormatType.UINT8)
            self.output_vstreams_params = OutputVStreamParams.make(self.network_group, format_type=FormatType.FLOAT32)
            
            self.infer_pipeline = self.stack.enter_context(
                InferVStreams(self.network_group, self.input_vstreams_params, self.output_vstreams_params)
            )
            self.stack.enter_context(self.network_group.activate(self.network_group_params))
            
            # --- JAVÍTÁS: HailoRT v4.23 API változás ---
            # A neveket most már közvetlenül a HEF modellből olvassuk ki
            self.input_name = self.hef.get_input_vstream_infos()[0].name
            self.output_name = self.hef.get_output_vstream_infos()[0].name
            
            self.get_logger().info("✅ Hailo-8 YOLOv8s KÉSZEN ÁLL A VADÁSZATRA!")
            
        except Exception as e:
            self.get_logger().error(f"❌ HAILO INICIALIZÁLÁSI HIBA: {e}")
            self.stack.close()

    def image_callback(self, msg):
        # Frame eldobása, ha az AI épp dolgozik
        if getattr(self, 'is_processing', False):
            return
            
        self.is_processing = True

        try:
            # 1. Kép átalakítása (ROS -> OpenCV)
            frame = self.bridge.imgmsg_to_cv2(msg, "rgb8") # A Hailo RGB-t szeret!
            
            # 2. YOLOv8 méretezés (640x640)
            orig_h, orig_w = frame.shape[:2]
            input_img = cv2.resize(frame, (640, 640))
            
            # 3. BEKÜLDÉS A HAILO CHIPBE (A Varázslat itt történik!)
            # Ez a művelet a CPU-t egyáltalán nem terheli, a 26 TOPS-os chip végzi!
            infer_results = self.infer_pipeline.infer({self.input_name: np.expand_dims(input_img, axis=0)})
            
            # 4. KIMENET FELDOLGOZÁSA
            detections = infer_results[self.output_name][0] 
            
            ai_cmd = Twist()
            best_person = None
            max_conf = 0.0
            
           # 4. KIMENET FELDOLGOZÁSA
            ai_cmd = Twist()
            best_person = None
            max_conf = 0.0
            
            # 1. A kőbe vésett (80, 5, 100) mátrix átvétele a chiptől
            raw_output = infer_results[self.output_name][0]
            
            # 2. Kivesszük a 0. osztályt (Ember). Ez GARANTÁLTAN egy (5, 100)-as fix mátrix.
            # 0. sor: ymin, 1. sor: xmin, 2. sor: ymax, 3. sor: xmax, 4. sor: confidence
            person_matrix = raw_output[0]
            
            # 3. Végigmegyünk a fix 100 helyen (Nincs dinamikus trükközés, nincs unpacking hiba)
            for i in range(100):
                conf = person_matrix[4][i]  # A 4. sor az i-edik oszlopban a magabiztosság
                
                # Ha a magabiztosság nagyobb mint 50% (azaz nem egy üres, nullákkal teli hely)
                if conf > 0.5:
                    if conf > max_conf:
                        max_conf = conf
                        ymin = person_matrix[0][i]
                        xmin = person_matrix[1][i]
                        ymax = person_matrix[2][i]
                        xmax = person_matrix[3][i]
                        best_person = (ymin, xmin, ymax, xmax, conf)
            
            # --- ROBOT MOZGATÁSA ---
            if best_person is not None:
                ymin, xmin, ymax, xmax, conf = best_person
                
                # YOLOv8 normalizált (0.0 - 1.0) koordináták visszaszorzása
                box_x = xmin * 640
                box_y = ymin * 640
                box_w = (xmax - xmin) * 640
                
                box_center_x = box_x + (box_w / 2)
                
                # --- PID SZABÁLYOZÁS ---
                error_yaw = self.center_x - box_center_x 
                ai_cmd.angular.z = error_yaw * self.kp_yaw
                
                error_linear = self.target_width - box_w
                ai_cmd.linear.x = error_linear * self.kp_linear
                
                # Sebességkorlátozás
                ai_cmd.angular.z = max(min(ai_cmd.angular.z, 0.5), -0.5)
                ai_cmd.linear.x = max(min(ai_cmd.linear.x, 0.5), -0.5)

                irany = "BALRA ⬅️" if ai_cmd.angular.z > 0 else "JOBBRA ➡️" if ai_cmd.angular.z < 0 else "KÖZÉPEN 🎯"
                tav = "ELŐRE ⬆️" if ai_cmd.linear.x > 0 else "HÁTRA ⬇️" if ai_cmd.linear.x < 0 else "ÁLL 🛑"
                
                self.get_logger().info(
                    f"🤖 LÁTOK EGY EMBERT! ({conf*100:.0f}%) -> Forgás: {irany}, Séta: {tav}"
                )
            else:
                self.get_logger().info("Nincs ember a láthatáron... 😴")

            # Parancs kiküldése az Agynak
            self.cmd_pub.publish(ai_cmd)
            
            ai_cmd = Twist()
            best_person = None
            max_conf = 0.0
            
            # Végigmegyünk a talált dobozokon
            for det in detections:
                ymin, xmin, ymax, xmax, conf, class_id = det
                
                # Ha a találat biztosabb mint 50%, ÉS a Class ID 0 (Ember)
                if conf > 0.5 and int(class_id) == 0:
                    if conf > max_conf:
                        max_conf = conf
                        best_person = det
            
            if best_person is not None:
                ymin, xmin, ymax, xmax, conf, class_id = best_person
                
                # A YOLOv8 normalizált (0.0 - 1.0) koordinátákat ad vissza, 
                # ezt visszaszorozzuk a 640-es méretre
                box_x = xmin * 640
                box_y = ymin * 640
                box_w = (xmax - xmin) * 640
                
                box_center_x = box_x + (box_w / 2)
                
                # --- PID SZABÁLYOZÁS A MOZGÁSHOZ ---
                error_yaw = self.center_x - box_center_x 
                ai_cmd.angular.z = error_yaw * self.kp_yaw
                
                error_linear = self.target_width - box_w
                ai_cmd.linear.x = error_linear * self.kp_linear
                
                ai_cmd.angular.z = max(min(ai_cmd.angular.z, 0.5), -0.5)
                ai_cmd.linear.x = max(min(ai_cmd.linear.x, 0.5), -0.5)

                irany = "BALRA ⬅️" if ai_cmd.angular.z > 0 else "JOBBRA ➡️" if ai_cmd.angular.z < 0 else "KÖZÉPEN 🎯"
                tav = "ELŐRE ⬆️" if ai_cmd.linear.x > 0 else "HÁTRA ⬇️" if ai_cmd.linear.x < 0 else "ÁLL 🛑"
                
                self.get_logger().info(
                    f"🤖 CÉLPONT BEMÉRVE! (Ember: {conf*100:.0f}%) -> "
                    f"Forgás: {irany}, Séta: {tav}"
                )
            else:
                self.get_logger().info("Nincs ember a láthatáron... 😴")

            # Parancs kiküldése az Agynak
            self.cmd_pub.publish(ai_cmd)

        except Exception as e:
            self.get_logger().error(f"Hiba a képfeldolgozás során: {e}")
            
        finally:
            self.is_processing = False

    def destroy_node(self):
        # Szabályos leállításkor elengedjük a PCIe eszközt!
        self.stack.close()
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