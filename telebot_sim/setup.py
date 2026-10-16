import os
from glob import glob
from setuptools import setup

package_name = 'telebot_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch/urdf/world files need to be installed to share/ or ros2 launch can't find them
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hammad Hassan',
    maintainer_email='hammaddammahss@gmail.com',
    description='Diff-drive robot sim with keyboard teleop in Gazebo',
    license='MIT',
    entry_points={
        'console_scripts': [
            'teleop_keyboard = telebot_sim.teleop_keyboard:main',
            'state_monitor = telebot_sim.state_monitor:main',
            'safety_controller = telebot_sim.safety_controller:main',
        ],
    },
)
