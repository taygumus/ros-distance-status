cmake_minimum_required(VERSION 2.8.3)
project(ros_project)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs nav_msgs tf message_filters message_generation dynamic_reconfigure)

add_service_files(
  FILES
  ComputeDistance.srv
)

add_message_files(
 FILES
 Status.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/parameters.cfg
)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(conversion_pub src/conversion_pub.cpp)
target_link_libraries(conversion_pub ${catkin_LIBRARIES})

add_executable(serve_distance src/serve_distance.cpp)
target_link_libraries(serve_distance ${catkin_LIBRARIES})
add_dependencies(serve_distance ${catkin_EXPORTED_TARGETS})

add_executable(request_distance src/request_distance.cpp)
target_link_libraries(request_distance ${catkin_LIBRARIES})
add_dependencies(request_distance ${catkin_EXPORTED_TARGETS})

add_executable(custom_message_pub src/custom_message_pub.cpp)
target_link_libraries(custom_message_pub ${catkin_LIBRARIES})
add_dependencies(custom_message_pub ${catkin_EXPORTED_TARGETS})
