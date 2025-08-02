from setuptools import setup
import os
from glob import glob

package_name = 'drl_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jiteshbhati',
    maintainer_email='j_bhati@mt.iitr.ac.in',
    description='Custom navigation stack built from scratch',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'global_planner = drl_navigation.global_planner:main',
            'controller = drl_navigation.controller:main',
            'goal_sender = drl_navigation.goal_sender:main',
        ],
    },
)
