from setuptools import setup, find_packages

package_name = 'um982_ros2_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/um982_driver.launch.py']),
        ('share/' + package_name + '/config', ['config/um982_params.yaml']),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'pyproj',
        'pyserial',
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your-email@example.com',
    description='ROS 2 driver for the UM982 GNSS module',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'um982_driver_node = um982_ros2_driver.node:main',
        ],
    },
)
