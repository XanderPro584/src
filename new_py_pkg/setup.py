from setuptools import find_packages, setup

package_name = 'new_py_pkg'

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
    maintainer_email='alexander@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "py_node = new_py_pkg.my_first_node:main",
            "robot_news_station = new_py_pkg.robot_news_station:main",
            "smartphone = new_py_pkg.smartphone:main",
            "number_publisher = new_py_pkg.number_publisher:main",
            "number_counter = new_py_pkg.number_counter:main",
            "add_two_ints_server = new_py_pkg.add_two_ints_server:main",
            "add_two_ints_client_no_oop = new_py_pkg.add_two_ints_client_no_oop:main",
            "add_two_ints_client = new_py_pkg.add_two_ints_client:main",
            "reset_number_count = new_py_pkg.reset_number_count:main",
            "hw_status_publisher = new_py_pkg.hw_status_publisher:main",
            "led_panel = new_py_pkg.led_panel:main",
            "battery_node = new_py_pkg.battery_node:main",
            "panel_state_listener = new_py_pkg.panel_state_listener:main",
        ],
    },
)
