from setuptools import setup

package_name = 'airsim_nav_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='naipi11',
    maintainer_email='naipi11@users.noreply.github.com',
    description='Sensor, clock and truth source for running a Gazebo-oriented ROS 2 navigation stack on AirSim.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'airsim_source = airsim_nav_bridge.source_node:main',
            'airsim_truth_recorder = airsim_nav_bridge.truth_recorder:main',
        ],
    },
)
