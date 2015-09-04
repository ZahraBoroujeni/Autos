cmake_minimum_required(VERSION 2.8.3)
project(test_gazebo)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS gazebo_ros)
# Depend on system install of Gazebo
find_package(gazebo REQUIRED)
## INCLUDE_DIRS:
include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
catkin_package(
    DEPENDS 
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

