from setuptools import find_packages, setup

package_name = 'crazyflie_ros2_multiranger_map_user'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'map_user = crazyflie_ros2_multiranger_map_user.map_user:main',
        ],
    },
)
