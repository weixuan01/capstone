from setuptools import find_packages, setup

package_name = 'crazyflie_ros2_multiranger_square'

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
    maintainer='weixuan',
    maintainer_email='poonweixuan@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'square_multiranger = crazyflie_ros2_multiranger_square.square_multiranger:main',
        ],
    },
)
