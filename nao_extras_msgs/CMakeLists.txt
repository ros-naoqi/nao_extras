cmake_minimum_required(VERSION 2.8.3)
project(nao_extras_msgs)

#List to make rest of code more readable
set( MESSAGE_DEPENDENCIES
    std_msgs
    actionlib_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    trajectory_msgs)

#Declare build dependencies
find_package(catkin REQUIRED 
    COMPONENTS
    message_generation
    ${MESSAGE_DEPENDENCIES} )

#Add message files
add_message_files(DIRECTORY msg/nao_face_detection
    FILES

    FaceDetected.msg 
    FaceInfo.msg 
    FaceExtraInfo.msg 
    FaceShapeInfo.msg 
    FaceEyePoints.msg 
    FaceEyebrowPoints.msg 
    FaceNosePoints.msg 
    FaceMouthPoints.msg 
)

#And now generate the messages
generate_messages(DEPENDENCIES ${MESSAGE_DEPENDENCIES})

#Declare package run-time dependencies
catkin_package( CATKIN_DEPENDS message_runtime ${MESSAGE_DEPENDENCIES})

