from setuptools import setup

package_name = "planning"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name, "planning/utils"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="team0",
    maintainer_email="design3-team3@proton.me",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "objective_manager_node = planning.objective_manager_node:main",
            "interface_bridge_node= planning.interface_bridge_node:main",
            "path_planning_node= planning.path_planning_node:main",
            "path_following_node= planning.path_following_node:main",
            "close_car_detection_node= planning.close_car_detection_node:main",
        ],
    },
)
