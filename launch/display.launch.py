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

        ## 5. AI Vision Node a kamerakép feldolgozásához
        # Node(
        #     package='my_hexapod',
        #     executable='ai_vision_node',
        #     name='ai_vision_node',
        #     output='screen'
        # ),

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
                {'publish_tf': False},
                {'gain': 0.05},       # <--- ÚJ: Alapértelmezés 0.1. Az 0.01 drasztikusan lecsillapítja a remegést!
                {'zeta': 0.005},       # <--- ÚJ: Megakadályozza a giroszkóp driftelését a háttérben.
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
                {'camera_info_url': 'file:///ros_ws/src/ros2-hexa/my_hexapod/config/camera_info.yaml'},
                
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
        
        # 13.8 Kálmán-szűrő (Szenzorfúzió: Lábak + IMU + Kamera)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                '/ros_ws/src/ros2-hexa/my_hexapod/config/ekf.yaml'
            ],
            remappings=[
                ('odometry/filtered', '/odom/filtered') # Ide küldi a tiszta adatot
            ]
        ),

        # 14. Az új AI Mélységérzékelő (Pszeudo RGB-D a Hailo chipről)
        Node(
            package='my_hexapod',
            executable='depth_vision_node',
            name='depth_vision_node',
            output='screen'
        ),

        # 15. RGB-D Szinkronizátor (Összepárosítja a színes képet és a mélységet)
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync',
            parameters=[
                {'approx_sync': True},
                {'approx_sync_max_interval': 0.05},
                {'queue_size': 20}
            ],
            remappings=[
                ('rgb/image', '/image_raw'),
                ('depth/image', '/depth/image_raw'),
                ('rgb/camera_info', '/camera_info'),
                ('rgbd_image', '/rgbd_image') # Ezt a kombinált csomagot küldi tovább
            ]
        ),
        
        # 15.5 AZ ÚJ LÁTÓIDEG: Vizuális Odometria (VIO)
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            parameters=[
                {'frame_id': 'base_link'},
                {'publish_tf': False},
                {'subscribe_rgbd': True},     
                
                # --- VÁLTOZTATÁSOK A SZIKLASZILÁRD FÉKHEZ ---
                {'guess_frame_id': 'odom'},   
                {'Odom/GuessMotion': 'false'},    # <--- ÚJ: NE puskázzon a lábakról! Ha áll a kép, mondja azt, hogy állunk.
                
                {'Vis/MaxFeatures': '600'},       # <--- ÚJ: 400 kevés volt, 1000 sok. A 600 az arany középút.
                {'Vis/EstimationType': '0'},      # <--- ÚJ: Vissza 3D->3D módba! Ez sokkal jobban bírja az AI "lélegző" mélységképét.
                {'Odom/UpdateTargetLinear': '0.01'} 
            ],
            remappings=[
                ('rgbd_image', '/rgbd_image'),
                ('imu', '/imu/data'),
                ('odom', '/odom/visual')  
            ]
        ),

        # 16. RTAB-Map (Immár igazi 3D VIO / RGB-D MÓDBAN!)
        Node(
            package='rtabmap_slam', 
            executable='rtabmap',
            name='rtabmap',
            parameters=[
                # Kikapcsoljuk a külön csatornákat, mert most már az "egybecsomagolt" rgbd_image-t használjuk
                {'subscribe_depth': False}, 
                {'subscribe_rgb': False},   
                {'subscribe_rgbd': True},   # <--- EZ KAPCSOLJA BE A 3D MÓDOT
                
                {'subscribe_odom': True},
                {'subscribe_imu': True},
                {'wait_imu_to_init': True},    
                
                {'frame_id': 'base_link'},
                {'odom_frame_id': 'odom'},
                {'approx_sync': True},         
                {'queue_size': 20},    
                
                {'Grid/RangeMax': '4.0'},            # Ne rajzoljon le semmit, ami 5 méternél messzebb van (ott már túl nagy az AI tévedése)
                {'Grid/CellSize': '0.05'},           # 5 cm-es felbontás (kisebb zaj, átláthatóbb térkép)
                {'Cloud/NoiseFilteringRadius': '0.1'}, # Kiszűri a "lebegő" magányos pontokat
                {'Cloud/NoiseFilteringMinNeighbors': '5'}, # Csak akkor tart meg egy pontot, ha vannak társai (tisztább kontúrok)      
                
                # --- SŰRŰBB TÉRKÉP PONTOK (A simább piros vonalért) ---
                {'RGBD/LinearUpdate': '0.01'},  # 10 cm helyett 1 cm-enként rakjon le egy pontot
                {'RGBD/AngularUpdate': '0.01'}, # 10 fok helyett már apró fordulásnál is rögzítsen
                {'Rtabmap/DetectionRate': '20'},# Ne várjon, pörgesse a térképfrissítést (alapból csak 1-5 Hz)  
                
                # --- TÉRKÉP TISZTÍTÁS ÉS SUGÁRKÖVETÉS ---
                {'Grid/3D': 'False'},               # Legyen igazi 3D-s a térkép rácsozata
                {'Grid/RayTracing': 'False'},       # Lőjön sugarakat a "szellemek" kitörléséhez
                {'Reg/Force3DoF': 'true'},  # Kényszerített 2D mód: X, Y és Z-forgás (Yaw) engedélyezett csak!
                
                # Szigorúbb Vizuális Odometria beállítások
                {'Vis/MinInliers': '15'},       # Minimum ennyi fix pont kell a mozgás felismeréséhez
                {'Vis/EstimationType': '0'},     # 3D->3D odometria számítás
            ],
            remappings=[
                ('rgbd_image', '/rgbd_image'), # A szinkronizátortól kapja a jelet!
                #('odom', '/odom/kinematic'),   # A lábak nyers odometriája (EKF helyett)
                ('odom', '/odom/filtered'),     # A szűrt odometria (EKF után)
                ('imu', '/imu/data'),         
            ],
            arguments=['-d'] 
        ),
    ])