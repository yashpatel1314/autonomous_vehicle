from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'av_sim'

setup(
    name=package_name,
    version='0.4.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.csv')),
        (os.path.join('share', package_name, 'urdf'),   glob('urdf/*.urdf')),
        (os.path.join('share', package_name),           ['pytest.ini']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Autonomous vehicle simulation — A* + Pure Pursuit in Gazebo Classic 11',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_manager   = av_sim.map_manager:main',
            'astar_planner = av_sim.astar_planner:main',
            'controller    = av_sim.controller:main',
        ],
    },
)
