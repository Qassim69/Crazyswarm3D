cmake_minimum_required(VERSION 3.8)
project(gaden_simulation_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Bout.msg"
  "msg/GenericLogData.msg"
  "msg/MapMSG.msg"
  "msg/StampedPosID.msg"
  "msg/ValPos.msg"
  "msg/ValPosStamped.msg"
  "srv/GetForces.srv"
  "srv/GetForcesNew.srv"
  "srv/GetValue.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs 
  ADD_LINTER_TESTS
)


ament_export_dependencies(rosidl_default_runtime)

ament_package()
