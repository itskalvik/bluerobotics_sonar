from setuptools import find_packages, setup

package_name = 'bluerobotics_sonar'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kalvik',
    maintainer_email='itskalvik@gmail.com',
    description='ROS 2 interface for Blue Robotics sonars',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ping1d = bluerobotics_sonar.ping1d:main',
            'ping1d_viewer = bluerobotics_sonar.ping1d_viewer:main',
            'ping360 = bluerobotics_sonar.ping360:main',
            'ping360_viewer = bluerobotics_sonar.ping360_viewer:main',
        ],
    },
)
