import os
from glob import glob

from setuptools import setup

package_name = 'jetnano_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'web'), glob('web/*.html')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='stevej52',
    maintainer_email='stevej52@gmail.com',
    description='Auto-detecting joystick teleoperation for the jetnano robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_node = jetnano_teleop.teleop_node:main',
            'list_devices = jetnano_teleop.list_devices:main',
            'web_teleop = jetnano_teleop.web_teleop:main',
        ],
    },
)
