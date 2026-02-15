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

        ### Én adtam hozzá
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        ### EDDIG ###
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
            'ik_node = my_hexapod.ik_node:main'
        ],
    },
)
