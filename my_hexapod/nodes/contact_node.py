import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray
import traceback

try:
    from gpiozero import Button
    IMPORT_SUCCESS = True
except ImportError:
    IMPORT_SUCCESS = False

class ContactNode(Node):
    def __init__(self):
        super().__init__('contact_node')
        
        # ROS Publisher: 6 elemű tömböt fogunk küldeni (1 = talajon, 0 = levegőben)
        self.contact_pub = self.create_publisher(Int8MultiArray, 'leg_contacts', 10)
        
        # 50 Hz-es frissítés (20ms), hogy elég gyors legyen a reakcióidő
        self.timer = self.create_timer(0.02, self.timer_callback)
        self.hardware_available = IMPORT_SUCCESS

        if self.hardware_available:
            try:
                # A kiválasztott GPIO pinek a 2x4-es klaszterből
                # Sorrend: Láb 1, Láb 2, Láb 3, Láb 4, Láb 5, Láb 6
                self.pins = [6, 12, 13, 19, 16, 26]
                
                self.switches = []
                for pin in self.pins:
                    # pull_up=True: Bekapcsoljuk a belső felhúzó ellenállást
                    # bounce_time=0.02: 20ms-es pergésmentesítés a zajok ellen
                    btn = Button(pin, pull_up=True, bounce_time=0.02)
                    self.switches.append(btn)
                    
                self.get_logger().info("✅ Talajérzékelő mikrokapcsolók inicializálva!")
            except Exception as e:
                self.get_logger().error(f"❌ GPIO Inicializálási hiba: {e}")
                self.get_logger().error(traceback.format_exc())
                self.hardware_available = False
        else:
            self.get_logger().warn("⚠️ gpiozero könyvtár hiányzik! Futtasd: pip install gpiozero")

    def timer_callback(self):
        msg = Int8MultiArray()
        
        if self.hardware_available:
            # Végigmegyünk a kapcsolókon. 
            # btn.is_pressed True, ha a kapcsoló lehúzza a lábat a GND-re (leért a láb)
            states = [1 if btn.is_pressed else 0 for btn in self.switches]
        else:
            # Dummy adatok, ha nincs hardver (minden láb a levegőben van)
            states = [0, 0, 0, 0, 0, 0]
            
        msg.data = states
        self.contact_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ContactNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()