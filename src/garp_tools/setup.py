from setuptools import find_packages, setup

package_name = 'garp_tools'

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
    maintainer='omerb',
    maintainer_email='omerb@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        "teleop_keyboard = garp_tools.teleop_keyboard:main",
        "odom_to_tf = garp_tools.odom_to_tf:main"
        ],

    },
)
