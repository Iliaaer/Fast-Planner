cmake_minimum_required(VERSION 2.8.3)
project(fast_drone)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  geographic_msgs
)

add_service_files(
  FILES
  takeoff.srv
  publishPose.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(arming src/arming.cpp)
target_link_libraries(arming ${catkin_LIBRARIES})
