cmake_minimum_required(VERSION 3.8)
project(diff_drive_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(gz-cmake3 REQUIRED)

# Harmonic
if("$ENV{GZ_VERSION}" STREQUAL "harmonic")
  find_package(gz-sim8 REQUIRED)
  set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
  message(STATUS "Compiling against Gazebo Harmonic")
endif()

# Check if we're building tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE) # Skip copyright linter
  set(ament_cmake_cpplint_FOUND TRUE)   # Skip cpplint
  ament_lint_auto_find_test_dependencies()
endif()

# Set model and world paths for Harmonic Gazebo
set(MODEL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/models/diff_drive")
set(WORLD_PATH "${CMAKE_CURRENT_SOURCE_DIR}/models/world")

# Install the models and worlds
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models)

# Install the launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

# Install the config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

# Install the Python executable
install(PROGRAMS
  src/diff_drive_robot/robot_mover.py  # Correct path to your Python script
  DESTINATION lib/${PROJECT_NAME}  # Installation directory
)

if("$ENV{GZ_VERSION}" STREQUAL "harmonic")
  # Any additional install commands for Harmonic-specific configurations
else()
  message(WARNING "GZ_VERSION is not set to 'harmonic', skipping Harmonic-specific configurations.")
endif()

ament_package()

