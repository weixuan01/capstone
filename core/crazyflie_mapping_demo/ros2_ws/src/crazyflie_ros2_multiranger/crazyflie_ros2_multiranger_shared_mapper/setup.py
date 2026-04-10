from setuptools import find_packages, setup

package_name = 'crazyflie_ros2_multiranger_shared_mapper'

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
    maintainer='Ryan Kwek',
    maintainer_email='ryankwek1@gmail.com',
    description='shared mapper for Crazyflie using odometry and multiranger data, to work with multiple drones',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'shared_mapper_multiranger = crazyflie_ros2_multiranger_shared_mapper.shared_mapper_multiranger:main',
        ],
    },
)
