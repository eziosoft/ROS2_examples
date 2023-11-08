from setuptools import find_packages, setup

package_name = 'test_pkg'

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
    maintainer_email='e@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_node = test_pkg.test_node:main",
            "test_publisher = test_pkg.test_publisher:main",
            "test_subscriber = test_pkg.test_subscriber:main",
            "test_service_server = test_pkg.test_service_server:main",
            "test_service_client = test_pkg.test_service_client:main"
        ],
    },
)
