from setuptools import find_packages, setup

package_name = "my_py_pkg"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kristin",
    maintainer_email="kristin@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "my_node = my_py_pkg.my_first_py_node:main",
            "robot_news_station = my_py_pkg.robot_news_station:main",
            "a_listener = my_py_pkg.a_listener:main",
            "number_publisher = my_py_pkg.number_publisher:main",
            "add_two_ints_server = my_py_pkg.add_two_ints_server:main",
            "add_two_ints_client_no_oop = my_py_pkg.add_two_ints_client_no_oop:main",
            "add_two_ints_client = my_py_pkg.add_two_ints_client:main",
            "hardware_status_publisher = my_py_pkg.hardware_status_publisher:main",
            "battery_node = my_py_pkg.battery_node:main",
        ],
    },
)
