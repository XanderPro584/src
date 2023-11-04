from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    robot_name_list = ["glskard", "BB8", "Daneel", "Lander", "C3P0"]

    package = "new_py_pkg"

    for name in robot_name_list:
        robot_news_node = Node(
            package=package,
            executable="robot_news_station",
            name=f"{name.lower()}_robot_news_station",
            parameters=[
                {"robot_name":name}
            ]
        )

        ld.add_action(robot_news_node)

    smartphone_node = Node(
        package=package,
        executable="smartphone"
    )

    ld.add_action(smartphone_node)

    return ld