from setuptools import setup

package_name = 'sensor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'sensor.stp32l',
        'sensor.gyrosensor',
        'sensor.sensor_publisher',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS2 package for sensor data aggregation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stp32l = sensor.stp32l:main',
            'gyrosensor = sensor.gyrosensor:main',
            'sensor_publisher = sensor.sensor_publisher:main',
        ],
    },
)
