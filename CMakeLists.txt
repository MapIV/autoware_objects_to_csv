cmake_minimum_required(VERSION 3.5)
project(tracked_objects_to_csv)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_auto_perception_msgs REQUIRED)

# Include C++ libraries
include_directories(include)

# Add executable
add_executable(tracked_objects_to_csv src/trackedobjectstocsv.cpp)

# Link libraries
ament_target_dependencies(tracked_objects_to_csv
  rclcpp
  autoware_auto_perception_msgs
  sensor_msgs
)

# Install targets
install(TARGETS
  tracked_objects_to_csv
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_package()