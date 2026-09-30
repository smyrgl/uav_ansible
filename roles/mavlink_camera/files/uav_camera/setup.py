from setuptools import setup
from glob import glob
setup(name='uav_camera', version='0.1.0', packages=['uav_camera'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/uav_camera']),
                  ('share/uav_camera', ['package.xml']),
                  ('share/uav_camera/config', glob('config/*'))],
      install_requires=['setuptools', 'pymavlink==2.4.49'], zip_safe=False,
      maintainer='John', maintainer_email='john@localhost',
      description='ROS2 native D555 RGB to NVENC RTSP and MAVLink Camera Protocol v2',
      license='Apache-2.0',
      entry_points={'console_scripts': ['camera = uav_camera.node:main']})
