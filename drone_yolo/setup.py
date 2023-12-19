from setuptools import find_packages, setup

package_name = 'drone_yolo'

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
            'webcam = drone_yolo.webcam:main',
            'yolov8_ros2_pt = drone_yolo.yolov8_ros2_pt:main',
        ],
    },
)
