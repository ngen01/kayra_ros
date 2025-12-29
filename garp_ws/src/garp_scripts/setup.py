from setuptools import find_packages, setup

package_name = 'garp_scripts'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='GARP Team',
    maintainer_email='garp@example.com',
    description='GARP AMR Python düğümleri - Teleop, Odometri, Navigasyon',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard = garp_scripts.teleop_keyboard:main',
            'odom_to_tf = garp_scripts.odom_to_tf:main',
            'waypoint_navigator = garp_scripts.waypoint_navigator:main',
            'differential_odometry = garp_scripts.differential_odometry:main',
        ],
    },
)
