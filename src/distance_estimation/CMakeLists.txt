cmake_minimum_required(VERSION 3.5)
project(distance_estimation)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cluster_msgs REQUIRED)
find_package(yolov8_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

add_executable(distance_estimation_node src/distance_estimation_node.cpp src/distance_estimation.cpp)

target_link_libraries( distance_estimation_node
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_LIBRARIES}
)

ament_target_dependencies(
  distance_estimation_node
  rclcpp std_msgs 
  sensor_msgs
  geometry_msgs
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  cluster_msgs
  yolov8_msgs
  message_filters
  OpenCV
) 

install(TARGETS
  distance_estimation_node
  DESTINATION lib/${PROJECT_NAME}
)

# # Files for testing
# install(FILES
#   test/cube1_record_1.yaml test/cube1_record_2.yaml test/lidar_detection_launch.testing.py 
#   DESTINATION lib/${PROJECT_NAME}
# )

# # Rviz2 config for visualization
# install(FILES config/lidar_detection.rviz 
#   DESTINATION lib/${PROJECT_NAME}
# )

# if(BUILD_TESTING)
#   #find_package(ament_lint_auto REQUIRED)
#   #ament_lint_auto_find_test_dependencies()
#   find_package(launch_testing_ament_cmake)    
#   add_launch_test(test/lidar_detection_launch.testing.py)
# endif()

ament_package()
