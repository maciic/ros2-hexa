import math

class HexapodGait:
    def __init__(self):
        # Járás paraméterek (default értékek, felülírhatóak)
        self.params = {
            'freq': 4.0,           # Sebesség (Hz)
            'step_len': 40.0,      # Lépéshossz (mm)
            'step_height': 30.0,   # Lépésmagasság (mm)
            'base_dist': 220.0,    # Alap terpesz
            'base_height': -100.0  # Test magasság
        }

    def get_tripod_phase(self, leg_key, t):
        """ Kiszámolja a láb aktuális fázisát (0..2PI) az idő és a csoport alapján. """
        # Tripod csoportosítás
        group_a = ["leg_1", "leg_3", "leg_5"]
        
        phase_offset = 0.0
        if leg_key not in group_a:
            phase_offset = math.pi # B csoport ellentétes fázisban
            
        return (t * self.params['freq']) + phase_offset

    def calculate_step_offset(self, phase, vel_x, vel_y, vel_yaw):
        """ 
        Generálja a lépés offszeteket (X, Y, Z, Turn) a fázis és sebesség alapján.
        """
        # Mozgunk, ha bármelyik irányba van parancs
        # (Itt egy kis 0.001 küszöböt használunk a lebegőpontos hibák ellen)
        is_moving = (abs(vel_x) > 0.001 or abs(vel_y) > 0.001 or abs(vel_yaw) > 0.001)

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
        if is_moving and math.sin(phase) > 0:
            off_z = math.sin(phase) * self.params['step_height']
            
        return off_walk, off_strafe, off_turn, off_z