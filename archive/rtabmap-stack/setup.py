from glob import glob

from setuptools import find_packages, setup

package_name = 'langrobo_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/scripts', glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rakhi24',
    maintainer_email='rohit.kalavapudi@gmail.com',
    description='LangRobo Jetson perception: nvblox + Nav2 (sim and real profiles)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'cmd_vel_stamper = langrobo_perception.cmd_vel_stamper:main',
            'detections_3d = langrobo_perception.detections_3d_node:main',
            'pixel_to_goal = langrobo_perception.pixel_to_goal_node:main',
            'sim_camera_relay = langrobo_perception.sim_camera_relay:main',
        ],
    },
)
