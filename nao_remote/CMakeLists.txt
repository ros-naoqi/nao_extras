cmake_minimum_required(VERSION 2.8.3)
project(nao_remote)

# Declare main C++ dependencies
set(ROS_CXX_DEPENDENCIES
    roscpp
    tf
    std_msgs
    std_srvs
    geometry_msgs
    nav_msgs
    move_base_msgs
    nao_msgs
    message_filters
    actionlib
    angles)

# Find catkin and all the required components
find_package(catkin REQUIRED
    COMPONENTS
    message_generation
    ${ROS_CXX_DEPENDENCIES} )

# Find Boost
find_package(Boost REQUIRED COMPONENTS signals)

# Set include and linking directories
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

# Generate services
add_service_files(DIRECTORY srv FILES SetTransform.srv )
generate_messages(DEPENDENCIES geometry_msgs)

# Declare info that other packages need to import messages generated here
catkin_package( CATKIN_DEPENDS message_runtime geometry_msgs )

# Create target: remap_odometry
#add_executable(remap_odometry src/remap_odometry.cpp)
#target_link_libraries(remap_odometry ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies(remap_odometry
#    ${nao_remote_EXPORTED_TARGETS} # Build services first
#    ${catkin_EXPORTED_TARGETS} )

# Create target: nao_path_follower
add_executable(nao_path_follower src/nao_path_follower.cpp)
target_link_libraries(nao_path_follower ${catkin_LIBRARIES})
add_dependencies(nao_path_follower ${catkin_EXPORTED_TARGETS})

# Installation rules for C++ targets
install(TARGETS
    #remap_odometry
    nao_path_follower
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )

# Install python scripts
install(DIRECTORY scripts/
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    USE_SOURCE_PERMISSIONS )

# Install launch files
install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch )

# Install config files
install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
