from glob import glob

from setuptools import find_packages, setup

package_name = 'algae_twin'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/maps', glob('maps/*')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
        ('share/' + package_name + '/models/burger_twin', glob('models/burger_twin/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aleksandar Kolev',
    maintainer_email='aleksanderkolev01@gmail.com',
    description='TurtleBot3 Burger digital twin (twin-only build): one Nav2 brain '
                'drives the real robot and its Gazebo twin in lockstep.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'twin_bridge = algae_twin.twin_bridge:main',
            'pose_sync = algae_twin.pose_sync:main',
            'map_edit = algae_twin.map_edit:main',
            'obstacle_mirror = algae_twin.obstacle_mirror:main',
            'mission = algae_twin.mission:main',
            'sim_battery = algae_twin.sim_battery:main',
            'operator_ui = algae_twin.operator_ui:main',
            'preflight = algae_twin.preflight:main',
        ],
    },
)
