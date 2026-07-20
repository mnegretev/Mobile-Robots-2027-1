from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'navig_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/'+ package_name + '/launch/', glob('launch/*')),
        ('share/'+ package_name + '/rviz/', glob('rviz/*')),
        ('share/'+ package_name + '/config/', glob('config/*')),
        ('share/'+ package_name + '/maps/', glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thedoctor',
    maintainer_email='marco.negrete@ingenieria.unam.edu',
    description='TODO: Package description',
    license='LGPL-3.0-only',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ros_basics_solved = navig_utils.ros_basics_solved:main',
            'ros_basics = navig_utils.ros_basics:main'
        ],
    },
)
