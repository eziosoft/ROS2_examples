from setuptools import find_packages, setup

package_name = 'my_robot1'

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
    maintainer='e',
    maintainer_email='eziosoft@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "camera_driver = test_pkg.camera_driver_node:main",
            "lidar_driver = test_pkg.lidar_driver_node:main",
            "lidar_emulator = test_pkg.lidar_emulator_node:main",
            "drive_node = test_pkg.drive_node:main",
        ],
    },
)
