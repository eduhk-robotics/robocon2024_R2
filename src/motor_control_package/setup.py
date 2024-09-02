from setuptools import setup

package_name = 'motor_control_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'pyserial'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Package for motor control communication with Arduino',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'motor_data_reader = motor_control_package.motor_data_reader:main',
        'motor_control_publisher = motor_control_package.motor_control_publisher:main',
        'motor_control_subscriber = motor_control_package.motor_control_subscriber:main',
        'angle_rotation = motor_control_package.angle_rotation:main',  # Add this line
        ],
    },

)