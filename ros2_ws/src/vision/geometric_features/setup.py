from setuptools import find_packages, setup

package_name = 'geometric_features'

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
            'canny = geometric_features.canny:main',
            'canny_solved = geometric_features.canny_solved:main',
            'hough = geometric_features.hough:main',
            'hough_solved = geometric_features.hough_solved:main',
            'houghP = geometric_features.houghP:main',
            'houghP_solved = geometric_features.houghP_solved:main',
        ],
    },
)
