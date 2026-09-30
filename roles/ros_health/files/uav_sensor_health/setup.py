from setuptools import find_packages, setup

setup(
    name="uav_sensor_health",
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/uav_sensor_health"]),
        ("share/uav_sensor_health", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="John",
    maintainer_email="john@example.com",
    description="Grouped sensor connection, data and timing diagnostics",
    license="Apache-2.0",
    entry_points={"console_scripts": ["sensor_health = uav_sensor_health.node:main"]},
)
