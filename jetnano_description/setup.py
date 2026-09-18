import os
from glob import glob

from setuptools import setup

package_name = 'jetnano_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='stevej52',
    maintainer_email='stevej52@gmail.com',
    description='URDF description for the jetnano four-wheel-steering robot',
    license='Apache-2.0',
    tests_require=['pytest'],
)
