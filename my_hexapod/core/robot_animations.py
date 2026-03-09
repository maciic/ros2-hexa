import math

class HexapodAnimations:
    def __init__(self):
        pass

    # --- ÚJ: JÁRÁSKOREOGRÁFIÁK (Fáziseltolások) ---
    def get_phase_offset(self, gait_mode, leg_key):
        """ 
        Visszaadja, hogy az adott lábnak mennyit kell késnie a fő ciklushoz képest.
        Itt dől el a sorrend! (Tripod vs Ripple)
        """
        phase_offset = 0.0
        
        if gait_mode == "TRIPOD":
            # Tripod csoportosítás
            group_a = ["leg_1", "leg_3", "leg_5"]
            if leg_key not in group_a:
                phase_offset = math.pi # B csoport ellentétes fázisban
                
        elif gait_mode == "RIPPLE":
            # Óramutató járásával megegyező hullámzó járás (1 -> 2 -> 3 -> 6 -> 5 -> 4)
            # A kört 6 egyenlő részre osztjuk (60 fokos, azaz pi/3 csúsztatások)
            order = {"leg_1": 0, "leg_2": 1, "leg_3": 2, "leg_6": 3, "leg_5": 4, "leg_4": 5}
            idx = order.get(leg_key, 0)
            phase_offset = idx * (math.pi / 3.0)
            
        return phase_offset

    # --- ANIMÁCIÓK ÉS PÓZOK ---

    def get_idle_breathing(self, t):
        """ 1. ÁLLAPOT: Alap lélegzés (Ha a robot áll és nem csinál semmit) """
        cycle_time = 3.0
        t_cycle = t % cycle_time
        breathe_z = math.sin((t_cycle / 2.0) * math.pi) * -15.0 if t_cycle < 2.0 else 0.0
        
        body_rpy = {'roll': 0.0, 'pitch': math.radians(-5.0), 'yaw': 0.0}
        return body_rpy, breathe_z, {} # A {} az üres láb-felülírás (senki sem integet)

    def get_attack_pose(self, t):
        """ 2. ÁLLAPOT: Harci póz némi fenékrázással és zihálással (készül a támadásra) """
        # Szinuszos "farokcsóválás" (Yaw forgás) az idő függvényében
        wag_yaw = math.sin(t * 15.0) * math.radians(2.0)
        
        # Szinuszos "zihálás" (fel-le mozgás Z tengelyen)
        breathe_z = 20.0 + math.sin(t * 5.0) * 5.0 
        
        # Bedöntjük -15 fokkal, és rátesszük a farokcsóválást
        body_rpy = {'roll': 0.0, 'pitch': math.radians(-15.0), 'yaw': wag_yaw}
        
        return body_rpy, breathe_z, {}

    def get_wave_pose(self, t):
        """ 3. ÁLLAPOT: Integetés. A 'leg_1' (Jobb első) a magasban kalimpál. """
        body_rpy = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        breathe_z = 0.0
        
        # Integető matek a jobb első lábnak
        wave_speed = 6.0  # Milyen gyorsan integessen
        wave_y = math.sin(t * wave_speed) * 40.0 # Jobbra-balra lengetés 40mm-en
        wave_z = -80.0 # Emelje fel a lábát 80mm-re a földtől
        
        # Ezt a parancsot az inverz kinematika csak az 1-es lábra fogja rátolni!
        leg_offsets = {
            'leg_1': {'x': 40.0, 'y': wave_y, 'z': wave_z} 
        }
        
        return body_rpy, breathe_z, leg_offsets