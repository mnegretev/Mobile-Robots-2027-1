from setuptools import find_packages, setup

package_name = 'path_planner'

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
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rrt = path_planner.rrt:main',
            'rrt_solved = path_planner.rrt_solved:main',
            'a_star = path_planner.a_star:main',
            'a_star_solved = path_planner.a_star_solved:main',
            'path_smoothing = path_planner.path_smoothing:main',
            'path_smoothing_solved = path_planner.path_smoothing_solved:main',
            'pot_fields = path_planner.pot_fields:main',
            'pot_fields_solved = path_planner.pot_fields_solved:main',
            
        ],
    },
)
