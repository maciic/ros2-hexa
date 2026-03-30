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

        # 5. AI Vision Node a kamerakép feldolgozásához
        Node(
            package='my_hexapod',
            executable='ai_vision_node',
            name='ai_vision_node',
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

        # 7. PS5 kontroller mapper
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
        ),
        
        # 10. IMU Node (Inertial Measurement Unit - Gyorsulásmérő és Giroszkóp)
        Node(
            package='my_hexapod',
            executable='imu_node',
            name='imu_node',
            output='screen'
        ),
        
        # ---------------------------------------
        # --- ÚJ NODE-OK A VIO/SLAM RENDSZERHEZ ---
        # ---------------------------------------

        # 11. IMU Szűrő (Madgwick) - Ezt eddig külön indítottad, most automatizáljuk!
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            parameters=[
                {'use_mag': False},
                {'publish_tf': False}
            ]
        ),

        # 12. Kamera node (A virtuális eszközzel és a kiszámolt Szemüveggel)
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera',
            parameters=[
                {'video_device': '/dev/video50'},
                # Hagyd meg a saját, jó útvonaladat a YAML-hoz!
                {'camera_info_url': 'file:///hexapod_workspace/src/ros2-hexa/my_hexapod/config/camera_info.yaml'},
                
                # --- ÚJ: FORMATÁLÁS AZ AI SZÁMÁRA ---
                {'output_encoding': 'rgb8'},   # Kijavítja a "Could not convert" errort!
                {'image_size': [640, 480]}     # Megvédi a Pi-t a túlterheléstől
            ]
        ),

        # 13. A Kamera "Nyakcsigolya"
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_static_tf',
            # A 'camera_link' helyett most már csak 'camera' legyen a vége!
            arguments=['0.074', '0.0', '0.081', '-1.5708', '0.0', '-1.3102', 'base_link', 'camera'] 
        ),
        
        # 13.5 Az IMU "Nyakcsigolyája" (Összeköti a testet az IMU-val a térben)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_static_tf',
            arguments=['0.0', '0.0', '0.048', '0.0', '0.0', '0.0', 'base_link', 'imu_link'] 
        ),

        # 14. RTAB-Map (A Vizuális Térképező és Odometria korrigáló AI)
        Node(
            package='rtabmap_slam', 
            executable='rtabmap',
            name='rtabmap',
            parameters=[
                {'subscribe_depth': False},
                {'subscribe_rgb': True},
                {'subscribe_odom': True},
                
                # --- AZ IMU INTEGRÁCIÓ PARAMÉTEREI ---
                {'subscribe_imu': True},       # BEKAPCSOLÓ GOMB: Figyeljen a belső fülre!
                {'wait_imu_to_init': True},    # Induláskor várja meg az első IMU adatot, ne induljon vakon.
                
                
                {'frame_id': 'base_link'},
                {'odom_frame_id': 'odom'},
                {'approx_sync': True},         
                {'queue_size': 20},            
                {'RGBD/AngularUpdate': '0.01'},
                {'RGBD/LinearUpdate': '0.01'},
                
                # --- TELJESÍTMÉNY MÓD ---
                {'Rtabmap/DetectionRate': '10'} # 2 Hz helyett 10 Hz: Folyamatos, sima kék vonal!
            ],
            remappings=[
                ('rgb/image', '/image_raw'),
                ('rgb/camera_info', '/camera_info'),
                ('odom', '/odom/kinematic'),
                ('imu', '/imu/data')           # Itt kapja meg a fizikai adatot
            ],
            arguments=['-d'] 
        ),
    ])