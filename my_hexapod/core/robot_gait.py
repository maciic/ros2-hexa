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
            'base_height': -100.0  # Test magasság
        }
        
        # 1. Behozzuk az animációs modult (A Koreográfust)
        self.animations = HexapodAnimations()
        
        # 2. Állapotgép változói (Ezeket már az Agy / robot_node diktálja!)
        self.current_state = "IDLE"  # Lehet: IDLE, WALK, ANIMATION
        self.active_animation = None # Ha van specifikus animáció (pl. "ATTACK")
        self.gait_mode = "TRIPOD"    # Lehet: TRIPOD, RIPPLE

    # FIGYELEM: Az update_state() függvényt VÉGLEG TÖRÖLTÜK! 
    # A sebesség alapú találgatás megszűnt, az Állapotot a brain_node mondja meg.

    def get_body_pose(self, t):
        """ Lekéri a test RPY dőlését és Z magasságát az aktuális állapottól függően """
        if self.current_state == "IDLE":
            return self.animations.get_idle_breathing(t)
            
        elif self.current_state == "WALK":
            # Séta közben megtartjuk a fenékemelést, de a lélegzés megáll
            # A harmadik visszatérési érték egy üres szótár ({}), mert séta közben senki sem integet
            return {'roll': 0.0, 'pitch': math.radians(0.0), 'yaw': 0.0}, 0.0, {}
            
        elif self.current_state == "ANIMATION":
            if self.active_animation == "ATTACK":
                return self.animations.get_attack_pose(t)
            elif self.active_animation == "WAVE":
                return self.animations.get_wave_pose(t)
            # Default fallback
            return {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}, 0.0, {}

    def get_leg_phase(self, leg_key, t):
        """ Kiszámolja a láb aktuális fázisát (0..2PI) az idő és a Koreográfus alapján. """
        
        # A Matematikus nem tudja mi az a Tripod vagy Ripple, csak elkéri a Koreográfustól a láb késését:
        phase_offset = self.animations.get_phase_offset(self.gait_mode, leg_key)
            
        # Tiszta matek: eltelt idő * frekvencia + a láb saját késése
        return (t * self.params['freq'] * 2 * math.pi) + phase_offset

    def calculate_step_offset(self, phase, vel_x, vel_y, vel_yaw):
        """ Generálja a lépés offszeteket. Csak akkor lép, ha WALK állapotban vagyunk! """
        if self.current_state != "WALK":
            return 0.0, 0.0, 0.0, 0.0

        # JAVÍTOTT VEKTOR NORMALIZÁLÁS (Már a forgást is belevesszük!)
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