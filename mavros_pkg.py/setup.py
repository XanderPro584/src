from setuptools import find_packages, setup

package_name = 'mavros_pkg'

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
            "takeoff_test = mavros_pkg.takeoff_test:main",
            "waypoint_test = mavros_pkg.waypoint_test:main",
            "start_mission = mavros_pkg.start_mission:main",
            "arm_test = mavros_pkg.arm_test:main", 
            "arm_test_oop = mavros_pkg.arm_test_oop:main",
            "sync_start_mission = mavros_pkg.sync_start_mission:main",
            "drone_controller = mavros_pkg.drone_controller:main",
            "drone_pose = mavros_pkg.drone_pose:main",
        ],
    },
)
