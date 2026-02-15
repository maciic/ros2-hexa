import math

class HexapodKinematics:
    def __init__(self, config):
        """
        A geometriai számítások osztálya.
        Bemenet: A teljes config dictionary.
        """
        dims = config['dimensions']
        self.coxaLength = dims['coxa_length']
        self.femurZOffset = dims['femur_z_offset']
        self.z_offset = dims['base_z_offset']
        
        # Femur & Tibia Matek (Előszámolva)
        self.femur_x = dims['femur_x']
        self.femur_drop = dims['femur_drop']
        self.femurLength_eff = math.sqrt(self.femur_x**2 + self.femur_drop**2)
        self.femur_angle_offset = math.atan2(self.femur_drop, self.femur_x)
        
        self.tibia_x = dims['tibia_x']
        self.tibia_drop = dims['tibia_drop']
        self.tibiaLength_eff = math.sqrt(self.tibia_x**2 + self.tibia_drop**2)
        self.tibia_angle_offset = math.atan2(self.tibia_drop, self.tibia_x)

    # === TEST KINEMATIKA SEGÉDFÜGGVÉNYEK ===
    
    def rotate_x(self, x, y, z, angle):
        """ Roll: Forgatás az X tengely körül (Y és Z változik) """
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        y_new = y * cos_a - z * sin_a
        z_new = y * sin_a + z * cos_a
        return x, y_new, z_new

    def rotate_y(self, x, y, z, angle):
        """ Pitch: Forgatás az Y tengely körül (X és Z változik) """
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        x_new = x * cos_a + z * sin_a
        z_new = -x * sin_a + z * cos_a
        return x_new, y, z_new

    def rotate_z(self, x, y, z, angle):
        """ Yaw: Forgatás a Z tengely körül (X és Y változik) """
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        x_new = x * cos_a - y * sin_a
        y_new = x * sin_a + y * cos_a
        return x_new, y_new, z

    def body_to_leg_coords(self, x_body, y_body, z_leg, leg_config, body_rpy):
        """
        Kiszámolja a láb végpontját a láb saját koordinátarendszerében.
        Tartalmazza: Body IK (forgatás) + Transzformáció a láb rögzítési pontjához.
        """
        # 1. BODY IK: TEST FORGATÁS ALKALMAZÁSA 🥋
        # A trükk: Ha a test dől, az olyan, mintha a láb koordinátáit elforgatnánk.
        rx, ry, rz = x_body, y_body, z_leg
        
        # Fontos: A sorrend számít! Általában: Yaw -> Pitch -> Roll
        rx, ry, rz = self.rotate_z(rx, ry, rz, body_rpy['yaw'])
        rx, ry, rz = self.rotate_y(rx, ry, rz, body_rpy['pitch'])
        rx, ry, rz = self.rotate_x(rx, ry, rz, body_rpy['roll'])
        
        # A végeredmény a transzformált láb pozíció (Globális, forgatott)
        final_x, final_y, final_z = rx, ry, rz

        # 2. ELTOLÁS (Globális -> Lokális konverzió a Mount ponthoz)
        dx = final_x - leg_config["mount_x"]
        dy = final_y - leg_config["mount_y"]
        
        # 3. FORGATÁS (Már radiánban van, nem kell átváltani!)
        theta = leg_config["mount_angle_rad"]
        
        # Forgatási mátrix (hogy a láb előre nézzen a saját rendszerében)
        x_leg = dx * math.cos(theta) + dy * math.sin(theta)
        y_leg = -dx * math.sin(theta) + dy * math.cos(theta)
        
        # Visszaadjuk a lokális koordinátákat és a forgatott Z-t
        # Illetve visszaadjuk a "Globális Forgatott" pontot is a vizualizációhoz (debug)
        return x_leg, y_leg, final_z, (final_x, final_y, final_z)

    def compute_ik(self, x, y, z):
        """ Inverz Kinematika egy lábra """
        # 1. ALAPSZÖG (Coxa elfordulása)
        fi = math.atan2(y, x)

        # 2. SÍKBELI TÁVOLSÁGOK KORRIGÁLÁSA
        R_total = math.sqrt(x**2 + y**2)
        R_effective = R_total - self.coxaLength
        
        # 3. MAGASSÁG KORRIGÁLÁSA
        actual_z = (self.z_offset - z) + self.femurZOffset

        # 4. A HÁROMSZÖG ÁTFOGÓJA (D)
        D = math.sqrt(R_effective**2 + actual_z**2)

        # Biztonsági limit
        max_reach = self.femurLength_eff + self.tibiaLength_eff
        if D > max_reach:
            D = max_reach - 0.001

        # === INNENTŐL UGYANAZ A MATEK ===
        gamma = math.atan2(actual_z, R_effective)

        val_alpha = (self.femurLength_eff**2 + D**2 - self.tibiaLength_eff**2) / (2 * self.femurLength_eff * D)
        alpha = math.acos(max(min(val_alpha, 1.0), -1.0))
        
        val_beta = (self.tibiaLength_eff**2 + self.femurLength_eff**2 - D**2) / (2 * self.femurLength_eff * self.tibiaLength_eff)
        beta  = math.acos(max(min(val_beta, 1.0), -1.0))

        # Kimenet
        fi_rad = fi
        alpha_rad = (gamma - alpha) - self.femur_angle_offset
        beta_rad = (math.pi - beta) - self.tibia_angle_offset

        return fi_rad, alpha_rad, beta_rad