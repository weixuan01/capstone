from setuptools import find_packages, setup

package_name = 'crazyflie_ros2_object_detection_scanner'

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
    maintainer='RyanKwek',
    maintainer_email='ryankwek1@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'object_detection_scanner = crazyflie_ros2_object_detection_scanner.object_detection_scanner:main',
        ],
    },
)
