import math
from .robot_animations import HexapodAnimations

class HexapodGait:
    def __init__(self):
        # Járás paraméterek (default értékek, felülírhatóak)
        self.params = {
            'freq': 0.7,           # Sebesség (Hz)
            'step_len': 120.0,     # Lépéshossz (mm)
            'step_height': 60.0,   # Lépésmagasság (mm)
            'base_dist': 300.0,    # Alap terpesz
            'base_height': -80.0  # Test magasság
        }
        
        # 1. Behozzuk az animációs modult (A Koreográfust)
        self.animations = HexapodAnimations()
        
        # 2. Állapotgép változói (Ezeket már az Agy / robot_node diktálja!)
        self.current_state = "IDLE"  # Lehet: IDLE, WALK, ANIMATION
        self.active_animation = None # Ha van specifikus animáció (pl. "ATTACK")
        self.gait_mode = "TRIPOD"    # Lehet: TRIPOD, RIPPLE

    def _cubic_bezier(self, p0, p1, p2, p3, t):
        """ Harmadfokú Bézier interpoláció egyetlen tengelyre (1D) """
        u = 1.0 - t
        return (u**3 * p0) + (3 * u**2 * t * p1) + (3 * u * t**2 * p2) + (t**3 * p3)

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

        # Vektor normalizálás (Ezt megtartjuk, ez tökéletes)
        magnitude = math.sqrt(vel_x**2 + vel_y**2 + vel_yaw**2)
        if magnitude > 1.0:
            vel_x = vel_x / magnitude
            vel_y = vel_y / magnitude
            vel_yaw = vel_yaw / magnitude

        # Alap amplitúdók (Mennyit mozduljon maximum előre/hátra)
        walk_amp = (self.params['step_len'] / 2.0) * vel_x
        strafe_amp = (self.params['step_len'] / 2.0) * vel_y
        turn_amp = (self.params['step_len'] / 2.0) * vel_yaw

        # Mennyi időt tölt a láb a földön? (pushFraction)
        # Tripodnál a ciklus felét (0.5) tölti a földön, Ripple-nél kicsit többet (pl. 0.66)
        push_fraction = 0.5 if self.gait_mode == "TRIPOD" else 0.66

        # Fázis (0-2PI) normalizálása 0.0 és 1.0 közötti 't' idővé
        t = (phase % (2 * math.pi)) / (2 * math.pi)

        off_walk, off_strafe, off_turn, off_z = 0.0, 0.0, 0.0, 0.0

        if t < push_fraction:
            # === STANCE FÁZIS (Láb a földön, tolja a robotot) ===
            # EGYENLETES sebességgel haladunk elölről (+amp) hátra (-amp)
            
            # t_ground: 0.0 -> 1.0-ig megy, amíg a láb a földön van
            t_ground = t / push_fraction 
            
            # Lineáris csúsztatás (Lerp) a kezdőpontból a végpontba
            off_walk = walk_amp - (2 * walk_amp * t_ground)
            off_strafe = strafe_amp - (2 * strafe_amp * t_ground)
            off_turn = turn_amp - (2 * turn_amp * t_ground)
            off_z = 0.0 # A láb szigorúan a földön marad
            
        else:
            # === SWING FÁZIS (Láb a levegőben, lép előre) ===
            # t_air: 0.0 -> 1.0-ig megy, amíg a láb a levegőben van
            t_air = (t - push_fraction) / (1.0 - push_fraction)
            
            # P0: Jelenlegi helyzet (hátul, a földön)
            p0_walk, p0_strafe, p0_turn, p0_z = -walk_amp, -strafe_amp, -turn_amp, 0.0
            
            # P1: Kiszakítás (Egyenesen felhúzzuk a kezdőponton)
            p1_walk, p1_strafe, p1_turn, p1_z = -walk_amp, -strafe_amp, -turn_amp, self.params['step_height']
            
            # P2: Átnyúlás (Előrelendítjük, magasan tartjuk)
            # Adunk neki 10% túllövést (overshoot), hogy határozottabb legyen a lépés
            overshoot = 1.2
            p2_walk = walk_amp * overshoot
            p2_strafe = strafe_amp * overshoot
            p2_turn = turn_amp * overshoot
            p2_z = self.params['step_height'] * 0.8 # Picit lejjebb engedjük, hogy szép íve legyen
            
            # P3: Célpont (Elöl, leér a földre)
            p3_walk, p3_strafe, p3_turn, p3_z = walk_amp, strafe_amp, turn_amp, 0.0

            # --- Bézier számítás minden dimenzióra ---
            off_walk = self._cubic_bezier(p0_walk, p1_walk, p2_walk, p3_walk, t_air)
            off_strafe = self._cubic_bezier(p0_strafe, p1_strafe, p2_strafe, p3_strafe, t_air)
            off_turn = self._cubic_bezier(p0_turn, p1_turn, p2_turn, p3_turn, t_air)
            off_z = self._cubic_bezier(p0_z, p1_z, p2_z, p3_z, t_air)

        return off_walk, off_strafe, off_turn, off_z