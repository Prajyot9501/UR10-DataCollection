import os
from glob import glob
from setuptools import setup

package_name = 'ur10e_data_collector'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A ROS2 package to collect UR10e joint states and publish data',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_collector = ur10e_data_collector.data_collector:main',
        ],
    },
    data_files=[
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
)
