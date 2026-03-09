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
        # 1. A Robot "agya", ami kiszámolja a pozíciókat (URDF / TF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
        ),

        # 2. AZ IZOMZAT (A mi új, letisztult robot_node-unk)
        Node(
            package='my_hexapod',
            executable='robot_node',
            name='robot_node',
            output='screen'
        ),

        # 3. Servo node (Hardveres kapcsolat)
        Node(
            package='my_hexapod',
            executable='servo_node',
            name='servo_node',
            output='screen'
        ),

        # 4. Battery node
        Node(
            package='my_hexapod',
            executable='battery_node',
            name='battery_node',
            output='screen'
        ),

        # 5. Contact / Szenzor node
        Node(
            package='my_hexapod',
            executable='contact_node',
            name='contact_node',
            output='screen'
        ),

        # 6. Joy Node (Kiolvassa a PS5 kontrollert a Bluetooth-ról)
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }]
        ),

        # --- ÚJ NODE-OK A SKYNET RENDSZERHEZ ---

        # 7. A Fordító (A régi teleop_node helyett. Gombok és karok feldolgozása)
        Node(
            package='my_hexapod',
            executable='ps5_mapper',
            name='ps5_mapper',
            output='screen'
        ),

        # 8. A Főnök (A Multiplexer, ami irányítja a forgalmat a kontroller és az izmok között)
        Node(
            package='my_hexapod',
            executable='brain_node',
            name='brain_node',
            output='screen'
        ),

        # ---------------------------------------

        # 9. A Foxglove híd
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen'
        )
    ])