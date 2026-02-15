import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import json
import os
import math

class ServoInterface(Node):
    def __init__(self):
        super().__init__('servo_interface')
        
        # Konfig betöltése
        script_dir = os.path.dirname(os.path.realpath(__file__))
        with open(os.path.join(script_dir, 'servo_config.json'), 'r') as f:
            self.config = json.load(f)
            
        self.servos = self.config['servos']
        
        # Feliratkozás a Matek Node-ra
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10
        )
        
        self.get_logger().info("Servo Interface Indul... Várakozás parancsokra.")

    def rad_to_pwm(self, angle_rad, servo_cfg):
        """ A nagy varázslat: Radián -> PWM konverzió """
        
        # 1. Radián -> Fok
        deg = math.degrees(angle_rad)
        
        # 2. Irány korrekció (ha fordítva van felszerelve a motor)
        deg = deg * servo_cfg['direction']
        
        # 3. PWM számítás
        # Egy átlagos szervó 1 fokra kb. 11.1 mikroszekundumot mozdul ((2500-500)/180)
        # De pontosabb, ha a tartományból számoljuk:
        pwm_per_deg = (servo_cfg['pwm_max'] - servo_cfg['pwm_min']) / servo_cfg['range_deg']
        
        # Az eltolás a középálláshoz (center) képest
        pwm_offset = deg * pwm_per_deg
        
        target_pwm = int(servo_cfg['center'] + pwm_offset)
        
        # 4. Biztonsági vágás (Limit)
        return max(min(target_pwm, servo_cfg['pwm_max']), servo_cfg['pwm_min'])

    def listener_callback(self, msg):
        # Végigmegyünk a kapott neveken (pl. joint_1_coxa)
        for i, name in enumerate(msg.name):
            if name in self.servos:
                # Megkeressük a hozzá tartozó szöget
                angle = msg.position[i]
                
                # Átszámoljuk PWM-re
                pwm = self.rad_to_pwm(angle, self.servos[name])
                
                # ITT KÜLDENÉNK A HARDVERNEK (Majd ide írjuk a PCA9685 kódot)
                # Teszthez csak kiírjuk:
                # self.get_logger().info(f"{name}: {angle:.2f} rad -> {pwm} us")

def main(args=None):
    rclpy.init(args=args)
    node = ServoInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()