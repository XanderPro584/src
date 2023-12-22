from setuptools import find_packages, setup

package_name = 'drone'

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
    maintainer='alexander',
    maintainer_email='agrawal.alexander@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_pose = drone.drone_pose:main',
            'waypoint_sender = drone.waypoint_sender:main',
            'global_pose_sub = drone.global_pose_sub:main',
            'drone_pose_copy = drone.drone_pose_copy:main',
            'local_pose_sub = drone.local_pose_sub:main',
            'drone_lifecycle_manager = drone.drone_lifecycle_manager:main',
            'search_and_rescue = drone.search_and_rescue:main',
            'orbit = drone.orbit:main',
            'ready_drone = drone.ready_drone:main',
            'go_to_waypoint = drone.go_to_waypoint:main',
            'drone_position = drone.drone_position:main',
            'set_cmd_vel = drone.set_cmd_vel:main',
        ],
    },
)
