from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'f112th_sim_2501_camel'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'description/diff_drive'), 
         glob(os.path.join('description/diff_drive', '*.[xacro]*'))),
        (os.path.join('share', package_name, 'description/ack_drive'), 
         glob(os.path.join('description/ack_drive', '*.[xacro]*'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.[world]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yaml]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DanielC',
    maintainer_email='dancorpa@gmail.com',
    description='F112th|2025-1 mobile robotics class project',
    license='N/N',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aeb_brake = f112th_sim_2501_camel.aeb_brake:main'
        ],
    },
)
