cmake_minimum_required(VERSION 3.0.2)
project(robotics_base)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_srvs
  geometry_msgs
  roscpp
  std_msgs
  tf
  tf2_ros
  tf2_eigen
  )
  
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  include
)

set(LIBRARIES
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)

add_executable(robotics_base src/robotics_base.cpp)
target_link_libraries(robotics_base ${LIBRARIES})

