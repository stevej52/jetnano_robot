import os
from glob import glob

from setuptools import setup

package_name = 'jetnano_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=['jetnano_navigation'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'behavior_trees'), glob('behavior_trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='stevej52',
    maintainer_email='stevej52@gmail.com',
    description='SLAM and Nav2 configuration for the jetnano robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'save_map = jetnano_navigation.save_map:main',
        ],
    },
)
