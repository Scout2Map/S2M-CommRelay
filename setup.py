import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'scout2map_comm'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'websockets'],
    zip_safe=True,
    maintainer='jihoonkim',
    maintainer_email='jihoonkim.tech@gmail.com',
    description='Communication relay for Scout2Map (buffered /events -> Web-Monitoring over WebSocket).',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'comm_relay_node = scout2map_comm.comm_relay_node:main',
        ],
    },
)
