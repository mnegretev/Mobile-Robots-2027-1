from setuptools import find_packages, setup

package_name = 'mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'k_means = mapping.k_means:main',
            'k_means_solved = mapping.k_means_solved:main',
            'cost_map = mapping.cost_map:main',
            'cost_map_solved = mapping.cost_map_solved:main',
        ],
    },
)
