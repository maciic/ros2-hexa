import json, math, os

def generate_urdf():
    # 1. Beolvassuk a konfigot
    # Megkeressük, hol van EZ a python fájl a lemezen és hozzáfűzzük a robot_config.json elérési útját
    script_dir = os.path.dirname(os.path.realpath(__file__))
    config_path = os.path.join(script_dir, 'robot_config.json')
    
    with open(config_path, 'r') as f:
        config = json.load(f)

    dims = config['dimensions']
    pkg = config['meta']['mesh_package']
    
    # Milliméter -> Méter váltás segédfüggvény
    m = lambda x: x / 1000.0

    # === URDF FEJLÉC ===
    urdf = """<?xml version="1.0"?>
<robot name="hexapod_bot">

  <material name="red"><color rgba="0.8 0 0 1"/></material>
  <material name="black"><color rgba="0.1 0.1 0.1 1"/></material>
  <material name="silver"><color rgba="0.75 0.75 0.75 1"/></material>

  <link name="base_link">
    <visual>
      <geometry><mesh filename="{}/base.stl" /></geometry>
      <material name="black"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </visual>
  </link>
""".format(pkg)

    # === LÁBAK GENERÁLÁSA ===
    for leg_key, leg_data in config['legs'].items():
        lid = leg_data['id']
        side = leg_data['side']
        
        # 1. COXA JOINT (A testhez rögzítés)
        # Az origin a configból jön
        urdf += f"""
  <joint name="joint_{lid}_coxa" type="revolute">
    <parent link="base_link"/>
    <child link="leg_{lid}_coxa"/>
    <origin xyz="{m(leg_data['mount_x'])} {m(leg_data['mount_y'])} 0" rpy="0 0 {leg_data['mount_angle_rad']}"/>
    <axis xyz="0 0 1"/>
    <limit lower="-{config['limits']['coxa']}" upper="{config['limits']['coxa']}" effort="1000" velocity="1.0"/>
  </joint>

  <link name="leg_{lid}_coxa">
    <visual>
      <geometry><mesh filename="{pkg}/coxa.stl"/></geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/> <material name="red"/>
    </visual>
  </link>
"""

        # 2. FEMUR JOINT
        # X eltolás = coxa_length
        # Z eltolás = femur_z_offset
        urdf += f"""
  <joint name="joint_{lid}_femur" type="revolute">
    <parent link="leg_{lid}_coxa"/>
    <child link="leg_{lid}_femur"/>
    <origin xyz="{m(dims['coxa_length'])} 0 {m(dims['femur_z_offset'])}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-{config['limits']['femur']}" upper="{config['limits']['femur']}" effort="1000" velocity="1.0"/>
  </joint>

  <link name="leg_{lid}_femur">
    <visual>
      <geometry><mesh filename="{pkg}/femur.stl"/></geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/> <material name="silver"/>
    </visual>
  </link>
"""

        # 3. TIBIA JOINT
        # Itt jön a trükk: Ha van FEMUR DROP, akkor a Tibia lejjebb kezdődik!
        # X = femur_x
        # Z = -femur_drop (lefelé negatív az URDF-ben a joint origin)
        urdf += f"""
  <joint name="joint_{lid}_tibia" type="revolute">
    <parent link="leg_{lid}_femur"/>
    <child link="leg_{lid}_tibia"/>
    <origin xyz="{m(dims['femur_x'])} 0 {-m(dims['femur_drop'])}" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-{config['limits']['tibia']}" upper="{config['limits']['tibia']}" effort="1000" velocity="1.0"/>
  </joint>

  <link name="leg_{lid}_tibia">
    <visual>
      <geometry><mesh filename="{pkg}/tibia.stl"/></geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/> <material name="black"/>
    </visual>
  </link>
"""

    # === LEZÁRÁS ===
    urdf += "</robot>"

    # Kiírás fájlba
    with open("./urdf/hexapod.urdf", 'w') as f:
        f.write(urdf)
    
    print("Siker! A hexapod.urdf legenerálva 6 lábbal.")

if __name__ == "__main__":
    generate_urdf()