from setuptools import setup

package_name = "control"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
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
            "motor_driver_node = control.motor_driver_node:main",
            "wall_following_node = control.wall_following_node:main",
            "null_command_sending_node = control.null_command_sending_node:main",
        ],
    },
)
