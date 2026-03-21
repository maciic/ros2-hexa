from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'my_hexapod'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maciic',
    maintainer_email='maciic@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [          
            
            'robot_node = my_hexapod.nodes.robot_node:main', # A Fő agy (Régi ik_node helyett)   
            'servo_node = my_hexapod.nodes.servo_node:main', # A hardveres interfész 
            'battery_node = my_hexapod.nodes.battery_node:main', # Az aksi állapota        
            'ps5_mapper = my_hexapod.nodes.ps5_mapper_node:main', # PS5 kontroller leképezése a robot parancsaira
            'brain_node = my_hexapod.nodes.brain_node:main', # Az agy, ami a vezérlő
            'ai_vision_node = my_hexapod.nodes.ai_vision_node:main', # A szem, ami látja a világot
            'calibration_node = my_hexapod.nodes.calibration_node:main', # A kalibráló, ami segít beállítani a szervókat
        ],
    },
)
