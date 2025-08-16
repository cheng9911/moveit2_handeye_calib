cmake_minimum_required(VERSION 3.5)

# Suppress CMP0002 warning 
option(CHECK_TIDY "Enable clang-tidy checks" OFF)
option(CMAKE_EXPORT_COMPILE_COMMANDS "Enable compile commands export" OFF)


project(franka_gazebo_bringup)

find_package(ament_cmake REQUIRED)

# Install project launch files
install(
  DIRECTORY
    launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
