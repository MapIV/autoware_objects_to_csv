cmake_minimum_required(VERSION 3.5)
project(objects_to_csv)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_auto_perception_msgs REQUIRED)
include_directories(include)

# Add executable
add_executable(tracked_objects_to_csv src/trackedobjectstocsv.cpp)
add_executable(detected_objects_to_csv src/detectedobjectstocsv.cpp)

# Link libraries
ament_target_dependencies(tracked_objects_to_csv
  rclcpp
  autoware_auto_perception_msgs
  sensor_msgs
)

ament_target_dependencies(detected_objects_to_csv
  rclcpp
  autoware_auto_perception_msgs
  sensor_msgs
)
# Install targets
install(TARGETS
  tracked_objects_to_csv
  detected_objects_to_csv
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()