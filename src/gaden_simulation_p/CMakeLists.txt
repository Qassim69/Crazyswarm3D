cmake_minimum_required(VERSION 3.8)
project(gaden_simulation_p)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# Install Python entry-point wrappers
install(PROGRAMS
  scripts/clock
  scripts/APF_1_1_env
  scripts/APF_1_1_agent
  DESTINATION lib/${PROJECT_NAME}
)

# Install data directories
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
