cmake_minimum_required(VERSION 2.8.3)
project(nao_teleop)

# List C++ dependencies on ros packages
set( ROS_CXX_DEPENDENCIES
    naoqi_bridge_msgs
    roscpp
    std_msgs
    sensor_msgs
    actionlib)

# Find catkin and all ROS required packages
find_package(catkin REQUIRED COMPONENTS ${ROS_CXX_DEPENDENCIES})

# Set include and linking directories
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

# Declare info that other packages need to import library generated here
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS ${ROS_CXX_DEPENDENCIES})
  
#Create target nao_teleop library
add_library(${PROJECT_NAME} src/teleop_nao_joy.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} )
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

#Create nao_teleop_node binary
add_executable(teleop_nao_joy src/teleop_nao_joy_node.cpp)
target_link_libraries(teleop_nao_joy ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(teleop_nao_joy ${catkin_EXPORTED_TARGETS})

# Installation instructions for library
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

# Installation instructions for library's include
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} )

# Installation instructions for binary
install(TARGETS teleop_nao_joy
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )

# Install launch files
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch )

# Install config files
install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)


