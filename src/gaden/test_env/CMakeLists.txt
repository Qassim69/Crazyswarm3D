cmake_minimum_required(VERSION 3.5)
project(test_env)
find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# Install scenario param/config files
install(DIRECTORY scenarios/ DESTINATION share/${PROJECT_NAME}/scenarios)

# Install navigation files
install(DIRECTORY navigation_config/ DESTINATION share/${PROJECT_NAME}/navigation_config)

ament_package()
