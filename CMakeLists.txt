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
  cv_bridge
  image_transport
)

# find_package(Boost REQUIRED COMPONENTS system signals)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated -Wdeprecated-declarations")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

message ( STATUS " CMake C++ FLAGS ${CMAKE_CXX_FLAGS}")

find_package(OpenCV)
find_package(Eigen3 REQUIRED)


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
add_executable(${PROJECT_NAME} src/main.cpp src/communication.cpp src/vehicle.cpp)


## Add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

