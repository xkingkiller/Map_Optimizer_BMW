cmake_minimum_required(VERSION 2.8.3)
project(map_optimizer_msg)


########################
##   Catkin Package   ##
########################

## Catkin packages required for the messages in this package
set( messages_DEPENDENCIES
    std_msgs
)

## Find catkin macros and libraries
## If COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz) is used, also find other catkin packages
## All build dependencies have to appear here
find_package( catkin REQUIRED COMPONENTS
    message_generation
    ${messages_DEPENDENCIES}
)

## Add messages here (files located at msg folder)
add_message_files(
    FILES
    OdomSelectList.msg
    SelectMode.msg
)

## Generate messages and services
## Specify the dependencies of the messages and services here
generate_messages( DEPENDENCIES ${messages_DEPENDENCIES} )

## Catkin package setup
## INCLUDE_DIRS - The exported include paths (i.e. cflags) for the package
## LIBRARIES - The exported libraries from the project
## CATKIN_DEPENDS - Other catkin projects that this project depends on (run dependencies)
## DEPENDS - Non-catkin CMake projects that this project depends on
catkin_package(
    CATKIN_DEPENDS
    message_runtime
    ${messages_DEPENDENCIES}
)


#################
##   Install   ##
#################


## Mark executable scripts for installation
#install( PROGRAMS
#    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark launch files for installation
#install( FILES
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#)

## Mark executables and/or libraries for installation
#install( TARGETS
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

