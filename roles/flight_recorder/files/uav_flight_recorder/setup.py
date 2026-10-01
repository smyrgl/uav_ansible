from setuptools import setup
setup(name='uav_flight_recorder', version='0.1.0', packages=['uav_flight_recorder'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/uav_flight_recorder']),
                  ('share/uav_flight_recorder', ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True, maintainer='John', maintainer_email='john@localhost',
      description='Flight-gated rosbag2 recorder', license='Apache-2.0',
      entry_points={'console_scripts': ['flight_recorder = uav_flight_recorder.node:main']})
