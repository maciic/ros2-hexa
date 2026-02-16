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
        
        Node(
            package='my_hexapod',
            executable='robot_node',
            name='robot_node',
            output='screen'
        ),
        
        Node(
            package='my_hexapod',
            executable='servo_node',
            name='servo_node',
            output='screen'
        ),
        
        # 3. A Foxglove híd
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen'
        )
    ])