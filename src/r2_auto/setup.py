from setuptools import find_packages, setup

package_name = 'r2_auto'

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
    maintainer='eduhkrobotic2',
    maintainer_email='eric0752805@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		"r2_ps4_node = r2_auto.r2_ps4:main",
		"r2_main_node = r2_auto.r2_main:main",
		"omni_pid = r2_auto.omni_pid:main",
        ],
    },
)
