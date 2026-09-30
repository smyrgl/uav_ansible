cmake_minimum_required(VERSION 3.8)
project(x950_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch meshes rviz urdf config docs
  DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
