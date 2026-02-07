cmake_minimum_required(VERSION 3.8)
project(racecar_interface)

# Warnings
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

#Packages needed from imports
find_package(builtin_interfaces REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

#Packages that were created
find_package(racecar_msgs REQUIRED)
find_package(racecar_routing REQUIRED)

#there are interfaces in this package
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Reset.srv"
  "srv/ResetObstacle.srv"
  DEPENDENCIES builtin_interfaces
)

# --- Python package install ---
ament_python_install_package(interface)

# Optional: Python executables
install(PROGRAMS
  scripts/visualization_node.py
  scripts/simulator_node.py
  scripts/traffic_simulation_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Data files
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

#launch file
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
