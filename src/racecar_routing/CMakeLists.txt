cmake_minimum_required(VERSION 3.8)
project(racecar_routing)

# Warnings
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Msg/srv deps referenced by Plan.srv
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# --- Interfaces ---
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Plan.srv"
  DEPENDENCIES nav_msgs geometry_msgs std_msgs builtin_interfaces
)

# --- Python package install ---
ament_python_install_package(routing)

# Optional: Python executables
install(PROGRAMS
  scripts/map_vis_node.py
  scripts/routing_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Data files
install(DIRECTORY
  maps
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
