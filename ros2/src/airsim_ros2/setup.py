from glob import glob
from setuptools import setup

package_name = 'airsim_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AirsimX maintainers',
    maintainer_email='naipi11@users.noreply.github.com',
    description='ROS 2 bridge for AirsimX using standard ROS messages',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'airsim_bridge = airsim_ros2.bridge:main',
            'airsim_sensor_bridge = airsim_ros2.sensor_bridge:main',
        ],
    },
)
