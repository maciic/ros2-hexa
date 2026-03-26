import math
from .robot_animations import HexapodAnimations

class HexapodGait:
    def __init__(self):
        self.params = {
            'freq': 0.7,           
            'step_len': 120.0,     
            'step_height': 60.0,   
            'base_dist': 300.0,    
            'base_height': -80.0  
        }
        
        self.animations = HexapodAnimations()
        self.current_state = "IDLE"  
        self.active_animation = None 
        self.gait_mode = "TRIPOD"    

    # --- ÚJ: A JÁRÁSMÓDOK KÖZPONTI PROFILJAI ---
    def _get_gait_profile(self):
        """ Visszaadja a kiválasztott járásmódhoz tartozó: (push_fraction, phase_offsets) értékeket """
        
        if self.gait_mode == "TRIPOD":
            # 51% a földön. Ballső, Jobb Közép, Bal Hátsó mozog együtt.
            return 0.51, {
                "leg_1": 0.0, "leg_3": 0.0, "leg_5": 0.0,
                "leg_2": 0.5, "leg_4": 0.5, "leg_6": 0.5
            }
            
        elif self.gait_mode == "RIPPLE":
            # ÚJ: 54% a földön. Ellentétes lábak lépnek utána (1/6 eltolással)
            return 0.54, {
                "leg_1": 0.0,                # Bal Első
                "leg_2": 1.0 / 6.0,          # Jobb Első
                "leg_3": 2.0 / 6.0,          # Bal Közép
                "leg_4": 3.0 / 6.0,          # Jobb Közép
                "leg_5": 4.0 / 6.0,          # Bal Hátsó
                "leg_6": 5.0 / 6.0           # Jobb Hátsó
            }
            
        elif self.gait_mode == "BI":
            # ÚJ: "Fura légy". 32% a földön. Mindig csak 2 láb érinti a talajt.
            return 0.32, {
                "leg_1": 0.0, "leg_6": 0.0,              # Csoport 1 (Bal Első + Jobb Hátsó)
                "leg_3": 1.0 / 3.0, "leg_4": 1.0 / 3.0,  # Csoport 2 (Bal Közép + Jobb Közép)
                "leg_5": 2.0 / 3.0, "leg_2": 2.0 / 3.0   # Csoport 3 (Bal Hátsó + Jobb Első)
            }
            
        elif self.gait_mode == "WAVE":
            # 83% a földön. Szépen körbemegy hullámban.
            return 5.0 / 6.0, {
                "leg_1": 0.0, "leg_2": 1.0/6.0, "leg_3": 2.0/6.0,
                "leg_6": 3.0/6.0, "leg_5": 4.0/6.0, "leg_4": 5.0/6.0
            }
            
        # Fallback
        return 0.5, {"leg_1": 0, "leg_2": 0, "leg_3": 0, "leg_4": 0, "leg_5": 0, "leg_6": 0}


    def _cubic_bezier(self, p0, p1, p2, p3, t):
        u = 1.0 - t
        return (u**3 * p0) + (3 * u**2 * t * p1) + (3 * u * t**2 * p2) + (t**3 * p3)

    def get_body_pose(self, t):
        if self.current_state == "IDLE":
            return self.animations.get_idle_breathing(t)
        elif self.current_state == "WALK":
            return {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}, 0.0, {}
        elif self.current_state == "ANIMATION":
            if self.active_animation == "ATTACK": return self.animations.get_attack_pose(t)
            elif self.active_animation == "WAVE": return self.animations.get_wave_pose(t)
            return {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}, 0.0, {}

    def get_leg_phase(self, leg_key, t):
        """ Kiszámolja a láb aktuális fázisát (0..2PI) az új beépített profilból """
        _, phase_offsets = self._get_gait_profile()
        phase_offset_normalized = phase_offsets.get(leg_key, 0.0)
        
        # Radiánra konvertáljuk (0.0-1.0 -> 0-2PI)
        phase_offset_rad = phase_offset_normalized * 2 * math.pi
        return (t * self.params['freq'] * 2 * math.pi) + phase_offset_rad

    def calculate_step_offset(self, phase, vel_x, vel_y, vel_yaw):
        if self.current_state != "WALK":
            return 0.0, 0.0, 0.0, 0.0

        magnitude = math.sqrt(vel_x**2 + vel_y**2 + vel_yaw**2)
        if magnitude > 1.0:
            vel_x, vel_y, vel_yaw = vel_x / magnitude, vel_y / magnitude, vel_yaw / magnitude

        walk_amp = (self.params['step_len'] / 2.0) * vel_x
        strafe_amp = (self.params['step_len'] / 2.0) * vel_y
        turn_amp = (self.params['step_len'] / 2.0) * vel_yaw

        # --- ÚJ: Itt is a beépített profilból kérjük le a push_fraction-t! ---
        push_fraction, _ = self._get_gait_profile()

        t = (phase % (2 * math.pi)) / (2 * math.pi)
        off_walk, off_strafe, off_turn, off_z = 0.0, 0.0, 0.0, 0.0

        if t < push_fraction:
            t_ground = t / push_fraction 
            off_walk = walk_amp - (2 * walk_amp * t_ground)
            off_strafe = strafe_amp - (2 * strafe_amp * t_ground)
            off_turn = turn_amp - (2 * turn_amp * t_ground)
            off_z = 0.0 
        else:
            t_air = (t - push_fraction) / (1.0 - push_fraction)
            p0_walk, p0_strafe, p0_turn, p0_z = -walk_amp, -strafe_amp, -turn_amp, 0.0
            p1_walk, p1_strafe, p1_turn, p1_z = -walk_amp, -strafe_amp, -turn_amp, self.params['step_height']
            
            overshoot = 1.2
            p2_walk = walk_amp * overshoot
            p2_strafe = strafe_amp * overshoot
            p2_turn = turn_amp * overshoot
            p2_z = self.params['step_height'] * 0.8 
            
            p3_walk, p3_strafe, p3_turn, p3_z = walk_amp, strafe_amp, turn_amp, 0.0

            off_walk = self._cubic_bezier(p0_walk, p1_walk, p2_walk, p3_walk, t_air)
            off_strafe = self._cubic_bezier(p0_strafe, p1_strafe, p2_strafe, p3_strafe, t_air)
            off_turn = self._cubic_bezier(p0_turn, p1_turn, p2_turn, p3_turn, t_air)
            off_z = self._cubic_bezier(p0_z, p1_z, p2_z, p3_z, t_air)

        return off_walk, off_strafe, off_turn, off_z