import math

class HexapodAnimations:
    def __init__(self):
        pass

    # --- ANIMÁCIÓK ÉS PÓZOK ---

    def get_idle_breathing(self, t):
        """ 1. ÁLLAPOT: Alap lélegzés (Ha a robot áll és nem csinál semmit) """
        cycle_time = 3.0
        t_cycle = t % cycle_time
        breathe_z = math.sin((t_cycle / 2.0) * math.pi) * 0.0 if t_cycle < 2.0 else 0.0
        
        body_rpy = {'roll': 0.0, 'pitch': math.radians(0.0), 'yaw': 0.0}
        return body_rpy, breathe_z, {} 

    def get_attack_pose(self, t):
        """ 2. ÁLLAPOT: Harci póz némi fenékrázással és zihálással """
        wag_yaw = math.sin(t * 15.0) * math.radians(2.0)
        breathe_z = 20.0 + math.sin(t * 5.0) * 5.0 
        body_rpy = {'roll': 0.0, 'pitch': math.radians(-15.0), 'yaw': wag_yaw}
        return body_rpy, breathe_z, {}

    def get_wave_pose(self, t):
        """ 3. ÁLLAPOT: Integetés. A 'leg_1' (Jobb első) a magasban kalimpál. """
        body_rpy = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        breathe_z = 0.0
        
        wave_speed = 6.0 
        wave_y = math.sin(t * wave_speed) * 70.0 
        wave_z = 300.0
        
        leg_offsets = {
            'leg_1': {'x': 60.0, 'y': wave_y, 'z': wave_z} 
        }
        
        return body_rpy, breathe_z, leg_offsets