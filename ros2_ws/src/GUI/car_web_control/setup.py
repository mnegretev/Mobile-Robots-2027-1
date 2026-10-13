from setuptools import setup
from glob import glob
import os

package_name = 'car_web_control'

setup(
    name=package_name,
    version='0.0.0',

    packages=[package_name],

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
        (
            os.path.join('share', package_name, 'web'),
            glob('web/*')
        ),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='user',
    maintainer_email='user@example.com',

    description='Web GUI for Ackermann vehicle control',
    license='Apache-2.0',

    entry_points={
        'console_scripts': [
            'web_server = car_web_control.web_server:main',
        ],
    },
)