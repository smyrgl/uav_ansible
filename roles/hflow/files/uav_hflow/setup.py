from setuptools import find_packages, setup

package_name = "uav_hflow"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="John Tumminaro",
    maintainer_email="smyrgl@gmail.com",
    description="Holybro H-Flow DroneCAN observer (listen-only) republishing PX4-typed ROS 2 topics",
    license="MIT",
    entry_points={"console_scripts": ["hflow_node = uav_hflow.node:main"]},
)
