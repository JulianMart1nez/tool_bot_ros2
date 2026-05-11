cmake_minimum_required(VERSION 3.8)
project(xarm5_gz_control)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install the Python package (for imports)
ament_python_install_package(${PROJECT_NAME})

# Install executable scripts into lib/xarm5_gz_control/
install(PROGRAMS
  scripts/keyboard_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install all data directories
install(DIRECTORY
  launch
  config
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

# ament package index marker
install(FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
