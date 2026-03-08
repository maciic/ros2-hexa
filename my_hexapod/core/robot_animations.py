import math

class HexapodAnimations:
    def __init__(self):
        # Ide jöhetnek majd az animációk sebesség/időzítés paraméterei
        pass

    def get_idle_breathing(self, t):
        """ 
        1. Állapot: Sima lélegző animáció és alap testtartás (fenékemelés) 
        Visszatérési érték: body_rpy (szótár), breathe_z (float)
        """
        cycle_time = 3.0
        t_cycle = t % cycle_time
        
        breathe_z = 0.0
        if t_cycle < 2.0:
            # -15.0 mm-es emelkedés szinuszosan
            breathe_z = math.sin((t_cycle / 2.0) * math.pi) * -15.0
            
        # Test dőlése: -5 fok pitch (a szoftveres fenékemelés)
        body_rpy = {'roll': 0.0, 'pitch': math.radians(-5.0), 'yaw': 0.0}
        
        return body_rpy, breathe_z

    def get_attack_pose(self, t):
        """ 
        2. Állapot (PÉLDA): Harci póz. Fej lent, fenék magasan!
        (Ezt később a kontroller egy gombjára kötjük)
        """
        # +10 fokos dőlés, és a test lejjebb engedése (-20mm)
        body_rpy = {'roll': 0.0, 'pitch': math.radians(10.0), 'yaw': 0.0}
        breathe_z = 20.0 
        
        return body_rpy, breathe_z