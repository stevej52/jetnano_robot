import os
from glob import glob

from setuptools import setup

package_name = 'jetnano_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=['jetnano_bringup'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('lib', package_name), ['scripts/cuvslam_vo.sh']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='stevej52',
    maintainer_email='stevej52@gmail.com',
    description='Bringup launch files for the jetnano robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_sensors = jetnano_bringup.fake_sensors:main',
            'tilt_guard = jetnano_bringup.tilt_guard:main',
        ],
    },
)
