cmake_minimum_required(VERSION 2.8.3)
project(nao_path_follower)

# Declare main C++ dependencies
set(ROS_CXX_DEPENDENCIES
  naoqi_bridge_msgs
  roscpp
  tf
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  move_base_msgs
  actionlib
  angles
)

# Find catkin and all the required components
find_package(catkin REQUIRED COMPONENTS
  ${ROS_CXX_DEPENDENCIES}
)

# Find Boost
find_package(Boost REQUIRED COMPONENTS signals)

# Set include and linking directories
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  CATKIN_DEPENDS ${ROS_CXX_DEPENDENCIES}
)

# Create target: nao_path_follower
add_executable(nao_path_follower src/nao_path_follower.cpp)
target_link_libraries(nao_path_follower ${catkin_LIBRARIES})
add_dependencies(nao_path_follower ${catkin_EXPORTED_TARGETS})

# Installation rules for C++ targets
install(TARGETS
  nao_path_follower
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

