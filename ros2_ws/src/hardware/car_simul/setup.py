from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'car_simul'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/'+ package_name + '/worlds/', glob('worlds/*.wbt')),
        ('share/'+ package_name + '/worlds/city_traffic_net/', glob('worlds/city_traffic_net/*')),
        ('share/'+ package_name + '/launch/', glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thedoctor',
    maintainer_email='marco.negrete@ingenieria.unam.edu',
    description='TODO: Package description',
    license='LGPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
