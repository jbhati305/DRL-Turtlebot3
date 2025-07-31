from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'drl_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Add map files
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jiteshbhati',
    maintainer_email='j_bhati@mt.iitr.ac.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'global_planner = drl_navigation.global_planner:main',
            'controller = drl_navigation.controller:main',
        ],
    },
)
