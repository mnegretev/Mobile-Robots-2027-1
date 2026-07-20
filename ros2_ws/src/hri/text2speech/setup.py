from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'text2speech'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), glob(os.path.join('models', '*.onnx'))),
        (os.path.join('share', package_name, 'models'), glob(os.path.join('models', '*.json'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robocup',
    maintainer_email='garcia.miguel.onate@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            't2s = text2speech.pipertts:main'
        ],
    },
)
