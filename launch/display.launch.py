import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'my_hexapod'
    urdf_file_name = 'hexapod.urdf'

    urdf_path = os.path.join(
        get_package_share_directory(package_name),
        'urdf',
        urdf_file_name)

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # 1. A Robot "agya", ami kiszámolja a pozíciókat
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
        ),
        # 2. A GUI ablak a csúszkákkal (EZT ADTUK HOZZÁ)
        # Node(
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     name='joint_state_publisher_gui',
        #     output='screen'
        # ),

        #Backup ik_node ha kellene még tesztelni
        # Node(
        #     package='my_hexapod',
        #     executable='ik_node_backup',
        #     name='  ik_node_backup',
        #     output='screen'
        # ),

        # Base ik_node
        Node(
            package='my_hexapod',
            executable='robot_node',
            name='robot_node',
            output='screen'
        ),

        #Servo node
        Node(
            package='my_hexapod',
            executable='servo_node',
            name='servo_node',
            output='screen'
        ),

        # Battery node
        Node(
            package='my_hexapod',
            executable='battery_node',
            name='battery_node',
            output='screen'
        ),

        # Contact / Szenzor node
        Node(
            package='my_hexapod',
            executable='contact_node',
            name='contact_node',
            output='screen'
        ),

        # 1. Joy Node (Kiolvassa a PS5 kontrollert)
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                # Ha a jstest alapján nem js0 volt, hanem pl. js1, akkor írd át itt!
                # 'dev': '/dev/input/js1', 
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }]
        ),

        # 2. Teleop Node (Lefordítja a karokat mozgás-parancsokká)
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[{
                'require_enable_button': False,  # Nem kell gombot nyomva tartani a mozgáshoz
                'axis_linear.x': 1,              # Bal kar Fel/Le -> Előre/Hátra
                'axis_linear.y': 0,              # Bal kar Balra/Jobbra -> Oldalazás
                'axis_angular.yaw': 3,           # Jobb kar Balra/Jobbra -> Forgás
                
                # --- ITT VANNAK A BŰVÖS SZORZÓK ---
                # Ne engedjük 1.0-ig! Így marad hely a kombinált mozgásnak.
                'scale_linear.x': 0.7,     # Maximum 70%-os előre/hátra lépés
                'scale_linear.y': 0.7,     # Maximum 70%-os oldalazás
                'scale_angular.yaw': 0.5,  # A forgást még jobban le is fojthatjuk (50%)
            }]
        ),

        # 3. A Foxglove híd
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen'
        )
    ])