import math

class HexapodAnimations:
    def __init__(self):
        self.sit_blend = 0.0  # Ezzel szabályozzuk az ereszkedés sebességét

    # --- ANIMÁCIÓK ÉS PÓZOK ---

    def get_idle_breathing(self, t):
        self.sit_blend = 0.0  # Töröljük a memóriát
        body_rpy = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        return body_rpy, 0.0, {} 

    def get_attack_pose(self, t):
        self.sit_blend = 0.0  # Töröljük a memóriát
        wag_yaw = math.sin(t * 15.0) * math.radians(2.0)
        breathe_z = 20.0 + math.sin(t * 5.0) * 5.0 
        body_rpy = {'roll': 0.0, 'pitch': math.radians(-15.0), 'yaw': wag_yaw}
        return body_rpy, breathe_z, {}

    def get_wave_pose(self, t):
        self.sit_blend = 0.0  # Töröljük a memóriát
        body_rpy = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        wave_y = math.sin(t * 6.0) * 70.0 
        leg_offsets = {'leg_1': {'x': 60.0, 'y': wave_y, 'z': 300.0}}
        return body_rpy, 0.0, leg_offsets

    def get_sit_pose(self, t):
        # Szépen lassan növeljük 0-ról 1.0-ra (kb. 1 másodperc alatt ér le)
        if self.sit_blend < 1.0:
            self.sit_blend = min(1.0, self.sit_blend + 0.02)
        
        # A +90 tolja le a testet a földig
        breathe_z = self.sit_blend * 95.0 
        
        # A lábak szétterpesztése arányosan az ereszkedéssel
        s = self.sit_blend * 0.0 
        leg_offsets = {
            'leg_1': {'x': s, 'y': s, 'z': 0.0},
            'leg_2': {'x': s, 'y': -s, 'z': 0.0},
            'leg_3': {'x': 0.0, 'y': s, 'z': 0.0},
            'leg_4': {'x': 0.0, 'y': -s, 'z': 0.0},
            'leg_5': {'x': -s, 'y': s, 'z': 0.0},
            'leg_6': {'x': -s, 'y': -s, 'z': 0.0}
        }
        
        body_rpy = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        return body_rpy, breathe_z, leg_offsets