import math
import time
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, TransformStamped

class HexapodOdometry:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = time.time()
        
        # Ezen fogjuk kiküldeni magát a vonalat!
        self.path_msg = Path()
        self.path_msg.header.frame_id = "odom" # Ezt a koordinátarendszert használjuk
        
        # Változók a hálózat kíméléséhez (csak akkor mentünk, ha lépett)
        self.last_path_x = 0.0
        self.last_path_y = 0.0
        self.path_save_distance = 0.02 # 2 centiméterenként rakunk le egy "kenyérmorzsát"

    def update_and_generate_messages(self, vel_x, vel_y, vel_yaw, freq, step_len, base_dist, is_walking, ros_clock_now):
        # --- VAK ODOMETRIA (DEAD RECKONING) SZÁMÍTÁSA ---
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Csak akkor integráljuk a mozgást, ha a robot tényleg sétál (nem idle vagy animáció)
        if is_walking:
            # A) Valós sebességek kiszámítása (m/s és rad/s)
            # step_len milliméterben van, ezért osztjuk 1000-rel!
            max_speed_m_s = (step_len / 1000.0) * freq
            
            # A maximális forgási sebesség a lépéshossz és a robot sugarának (base_dist) arányából jön
            max_turn_rad_s = (step_len / base_dist) * freq
            
            vx = vel_x * max_speed_m_s
            vy = vel_y * max_speed_m_s
            vyaw = vel_yaw * max_turn_rad_s

            # B) Sebességvektorok elforgatása a robot aktuális globális (Yaw) szögéhez
            # Ez felel azért, hogy ha a robot elfordul 90 fokot, akkor az "előre" már a globális Y tengely legyen.
            dx = (vx * math.cos(self.yaw) - vy * math.sin(self.yaw)) * dt
            dy = (vx * math.sin(self.yaw) + vy * math.cos(self.yaw)) * dt
            dyaw = vyaw * dt

            # C) Pozíciók frissítése (Integrálás)
            self.x += dx
            self.y += dy
            self.yaw += dyaw

        # D) Odometria üzenet összeállítása
        odom_msg = Odometry()
        odom_msg.header.stamp = ros_clock_now
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Pozíció (X, Y)
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Yaw szög átváltása Kvaternióba (Z és W tengelyek használatával)
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)

        # --- TF2 TRANSFORM ÜZENET ---
        # Ez köti össze a Foxglove-ban a vonalat a robottal!
        tf_msg = TransformStamped()
        tf_msg.header.stamp = ros_clock_now
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_link"
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = odom_msg.pose.pose.orientation

        # --- A Vonal (Path) építése ---
        path_updated = False
        dist_moved = math.sqrt((self.x - self.last_path_x)**2 + (self.y - self.last_path_y)**2)
        
        # Csak akkor adjuk hozzá a listánkhoz, ha mozdult 2 centit, hogy ne fagyjon le a Foxglove
        if dist_moved >= self.path_save_distance:
            # 1. Készítünk egy "Időbélyegzett Pózt" a jelenlegi helyzetből
            pose = PoseStamped()
            pose.header.stamp = ros_clock_now
            pose.header.frame_id = "odom"
            pose.pose = odom_msg.pose.pose
            
            # 2. Hozzáadjuk a listánkhoz
            self.path_msg.poses.append(pose)
            self.path_msg.header.stamp = ros_clock_now
            
            self.last_path_x = self.x
            self.last_path_y = self.y
            path_updated = True
            
            # 3. Biztonsági korlát: Ne egye meg a Pi összes RAM-ját! 
            # Csak az utolsó 500 pontot tartjuk meg.
            if len(self.path_msg.poses) > 500:
                self.path_msg.poses.pop(0)

        return odom_msg, tf_msg, self.path_msg, path_updated