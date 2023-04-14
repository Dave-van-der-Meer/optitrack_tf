from setuptools import setup
from glob import glob
import os

package_name = 'optitrack_tf'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dave van der Meer',
    maintainer_email='dave.vandermeer@uni.lu',
    description='Converts Pose messages published by the VRPN package to TF messages',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'broadcast_tf = optitrack_tf.broadcast_tf:main',
        ],
    },
)
