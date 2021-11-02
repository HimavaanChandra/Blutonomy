cmake_minimum_required(VERSION 2.8.3)
project(Blutonomy)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  sensor_msgs
  std_msgs
  roslib
  geometry_msgs
  nav_msgs
  tf
  tf2
  cv_bridge
  image_transport
)

# find_package(Boost REQUIRED COMPONENTS system signals)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated -Wdeprecated-declarations")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

message ( STATUS " CMake C++ FLAGS ${CMAKE_CXX_FLAGS}")

find_package(OpenCV)
find_package(Eigen3 REQUIRED)

 add_message_files(
   FILES
   data_packet.msg
 )

 generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
 )
 
catkin_package(
LIBRARIES
INCLUDE_DIRS
CATKIN_DEPENDS
roscpp
sensor_msgs
std_msgs
tf
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable
add_executable(${PROJECT_NAME} src/main.cpp src/vehicle.cpp src/marker_helper.cpp)
# add_executable(data_packet_pub src/data_packet_pub.cpp)
# add_executable(data_packet_sub src/data_packet_sub.cpp)
add_executable(wamv_tf_publisher src/wamv_tf_publisher.cpp)

## Add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(data_packet_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(data_packet_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
 target_link_libraries(wamv_tf_publisher
   ${catkin_LIBRARIES}
 )
