from setuptools import setup

package_name = "perception"

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
            "car_detection_node = perception.car_detection_node:main",
            "optical_sensor_node= perception.optical_sensor_node:main",
        ],
    },
)
