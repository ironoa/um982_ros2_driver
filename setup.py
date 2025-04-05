from setuptools import find_packages, setup

package_name = 'um982_ros2_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/um982_ros2_driver.launch.py']),
    ],
    install_requires=['setuptools', 'pyproj', 'pyserial'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='ROS2 driver for UNICORECOMM UM982/UM980 GPS',
    license='GPL',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "um982_node = um982_ros2_driver.um982_node:main",
        ],
    },
)
