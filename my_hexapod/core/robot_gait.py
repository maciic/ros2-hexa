import math
from .robot_animations import HexapodAnimations

class HexapodGait:
    def __init__(self):
        # Járás paraméterek (default értékek, felülírhatóak)
        self.params = {
            'freq': 1.0,           # Sebesség (Hz)
            'step_len': 100.0,     # Lépéshossz (mm)
            'step_height': 40.0,   # Lépésmagasság (mm)
            'base_dist': 250.0,    # Alap terpesz
            'base_height': -100.0   # Test magasság
        }
        
        # 1. Behozzuk az animációs modult
        self.animations = HexapodAnimations()
        
        # 2. Állapotgép változói
        self.current_state = "IDLE"  # Lehet: IDLE, WALK, ANIMATION
        self.active_animation = None # Ha van specifikus animáció (pl. "ATTACK")

    def update_state(self, vel_x, vel_y, vel_yaw):
        """ Eldönti, hogy épp mit csináljon a robot a parancsok alapján """
        
        # Ha be van kapcsolva egy fix animáció, az mindent felülír
        if self.active_animation is not None:
            self.current_state = "ANIMATION"
        
        # Ha nincs animáció, de van sebességparancs -> SÉTA
        elif abs(vel_x) > 0.1 or abs(vel_y) > 0.1 or abs(vel_yaw) > 0.1:
            self.current_state = "WALK"
            
        # Különben -> PIHENÉS (Lélegzés)
        else:
            self.current_state = "IDLE"

    def get_body_pose(self, t):
        """ Lekéri a test RPY dőlését és Z magasságát az aktuális állapottól függően """
        if self.current_state == "IDLE":
            return self.animations.get_idle_breathing(t)
            
        elif self.current_state == "WALK":
            # Séta közben megtartjuk a fenékemelést, de a lélegzés megáll
            return {'roll': 0.0, 'pitch': math.radians(0.0), 'yaw': 0.0}, 0.0
            
        elif self.current_state == "ANIMATION":
            if self.active_animation == "ATTACK":
                return self.animations.get_attack_pose(t)
            # Default fallback
            return {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}, 0.0

    def get_tripod_phase(self, leg_key, t):
        """ Kiszámolja a láb aktuális fázisát (0..2PI) az idő és a csoport alapján. """
        # Tripod csoportosítás
        group_a = ["leg_1", "leg_3", "leg_5"]
        
        phase_offset = 0.0
        if leg_key not in group_a:
            phase_offset = math.pi # B csoport ellentétes fázisban
            
        return (t * self.params['freq'] * 2 * math.pi) + phase_offset

    def calculate_step_offset(self, phase, vel_x, vel_y, vel_yaw):
        """ Generálja a lépés offszeteket. Csak akkor lép, ha WALK állapotban vagyunk! """
        if self.current_state != "WALK":
            return 0.0, 0.0, 0.0, 0.0

        # --- JAVÍTOTT VEKTOR NORMALIZÁLÁS (Már a forgást is belevesszük!) ---
        magnitude = math.sqrt(vel_x**2 + vel_y**2 + vel_yaw**2)
        if magnitude > 1.0:
            vel_x = vel_x / magnitude
            vel_y = vel_y / magnitude
            vel_yaw = vel_yaw / magnitude

        # 1. SÉTA (Walk) - X tengely
        walk_amp = (self.params['step_len'] / 2.0) * vel_x
        off_walk = -math.cos(phase) * walk_amp

        # 2. OLDALAZÁS (Strafe) - Y tengely
        strafe_amp = (self.params['step_len'] / 2.0) * vel_y
        off_strafe = -math.cos(phase) * strafe_amp

        # 3. FORGÁS (Turn) - Érintő irány (csak a hossza)
        turn_amp = (self.params['step_len'] / 2.0) * vel_yaw
        off_turn = -math.cos(phase) * turn_amp

        # 4. EMELÉS (Lift) - Z tengely
        off_z = 0.0
        if math.sin(phase) > 0:
            off_z = math.sin(phase) * self.params['step_height']
            
        return off_walk, off_strafe, off_turn, off_z