import time
from geometry_msgs.msg import Twist

class AIStateMachine:
    def __init__(self):
        # --- KÖVETÉSI BEÁLLÍTÁSOK ---
        self.target_hip_width = 80 
        self.kp_linear = 0.005
        self.kp_yaw = 0.002
        self.center_x = 320 
        
        # --- ROBOTIKAI MEMÓRIA ---
        self.last_anim_sent = "NONE"
        self.last_seen_time = 0.0
        self.memory_duration = 1.0  
        self.active_gesture = "STOP"
        self.last_mode = "STANDBY"

        # --- ÚJ: GESZTUS ZÁR (PERGÉSMENTESÍTÉS) ---
        self.gesture_lock_time = 0.0
        self.gesture_min_duration = 1.0  # MÁSODPERC: Minimum ennyi ideig lejátsza az animációt

    def process(self, current_mode, keypoints):
        cmd_vel = Twist()
        anim_request = "NONE"
        now = time.time()

        # --- MÓDVÁLTÁS ÉRZÉKELÉSE ÉS TISZTÍTÁS ---
        if current_mode != self.last_mode:
            self.last_mode = current_mode
            self._reset_memory()
            return self._filter_output(cmd_vel, "STOP")

        # --- 0. POWER GATING (ALVÓ MÓD) ---
        if current_mode == "STANDBY":
            self._reset_memory()
            return self._filter_output(cmd_vel, "NONE")

        # --- 1. MEMÓRIA FRISSÍTÉSE ---
        if keypoints is not None:
            self.last_seen_time = now

        is_remembered = (now - self.last_seen_time) < self.memory_duration

        # --- 2. HA TÉNYLEG ELVESZTETTÜK AZ EMBERT ---
        if not is_remembered:
            self.active_gesture = "STOP"
            if current_mode == "SENTRY":
                cmd_vel.angular.z = 0.2  
                anim_request = "STOP"    
            else:
                anim_request = "STOP"    
            
            return self._filter_output(cmd_vel, anim_request)

        # --- 3. HA LÁTJUK (VAGY EMLÉKSZÜNK RÁ) ---
        if keypoints is None:
            if current_mode == "GESTURE": anim_request = self.active_gesture
            elif current_mode == "SENTRY": anim_request = "ATTACK"
            return self._filter_output(cmd_vel, anim_request)

        # Ha van tiszta, friss képadat:
        l_shoulder = keypoints.get(5)
        r_shoulder = keypoints.get(6)
        l_wrist = keypoints.get(9)
        r_wrist = keypoints.get(10)
        l_hip = keypoints.get(11)
        r_hip = keypoints.get(12)

        # -- A. KUTYUS MÓD (FOLLOW) --
        if current_mode == "FOLLOW":
            if l_hip and r_hip and l_hip['conf'] > 0.4 and r_hip['conf'] > 0.4:
                current_hip_width = abs(r_hip['x'] - l_hip['x'])
                error_linear = self.target_hip_width - current_hip_width
                person_center_x = (l_hip['x'] + r_hip['x']) / 2.0
                error_yaw = self.center_x - person_center_x

                if abs(error_linear) < 15: error_linear = 0.0
                if abs(error_yaw) < 30: error_yaw = 0.0

                cmd_vel.linear.x = max(min(error_linear * self.kp_linear, 0.3), -0.3)
                cmd_vel.angular.z = max(min(error_yaw * self.kp_yaw, 0.4), -0.4)
            else:
                anim_request = "STOP"

        # -- B. JEDI MÓD (GESTURE) --
        elif current_mode == "GESTURE":
            intended_gesture = "STOP" # Alapértelmezett feltételezés: nem csinál semmit

            if l_shoulder and r_shoulder and l_wrist and r_wrist and \
               l_shoulder['conf'] > 0.5 and r_shoulder['conf'] > 0.5 and \
               l_wrist['conf'] > 0.5 and r_wrist['conf'] > 0.5:
                
                right_hand_up = r_wrist['y'] < r_shoulder['y']
                left_hand_up = l_wrist['y'] < l_shoulder['y']

                if right_hand_up and left_hand_up:
                    intended_gesture = "STOP" 
                elif right_hand_up and not left_hand_up:
                    intended_gesture = "WAVE"
                elif left_hand_up and not right_hand_up:
                    intended_gesture = "ATTACK"

            # --- ÚJ: GESZTUS ZÁR LOGIKA ---
            # Ha új, aktív gesztust mutatunk neki (WAVE vagy ATTACK)
            if intended_gesture in ["WAVE", "ATTACK"] and intended_gesture != self.active_gesture:
                self.active_gesture = intended_gesture
                self.gesture_lock_time = now # Elindítjuk a stoppert!
            
            # Ha a kamera szerint meg kéne állni (letette a kezét, vagy pislog a szenzor)
            elif intended_gesture == "STOP":
                # Csak akkor engedjük megállni, ha már letelt az 1 másodperc!
                if (now - self.gesture_lock_time) >= self.gesture_min_duration:
                    self.active_gesture = "STOP"
            
            anim_request = self.active_gesture

        # -- C. ŐRSZEM MÓD (SENTRY) --
        elif current_mode == "SENTRY":
            anim_request = "ATTACK"

        return self._filter_output(cmd_vel, anim_request)

    def _filter_output(self, cmd_vel, anim_request):
        anim_cmd_to_send = "NONE"
        if anim_request != self.last_anim_sent:
            anim_cmd_to_send = anim_request
            self.last_anim_sent = anim_request
        return cmd_vel, anim_cmd_to_send

    def _reset_memory(self):
        self.last_anim_sent = "NONE"
        self.active_gesture = "STOP"
        self.last_seen_time = 0.0
        self.gesture_lock_time = 0.0 # Stoppert is nullázzuk