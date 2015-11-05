cmake_minimum_required(VERSION 2.8.3)
project(my_code)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg geometry_msgs)
find_package(Boost REQUIRED COMPONENTS thread)

find_package(Threads REQUIRED)
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})


# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
 generate_messages(
   DEPENDENCIES
   std_msgs
)
catkin_package(
    DEPENDS Boost
    CATKIN_DEPENDS roscpp 
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO

)
add_executable(talker src/camera_test.cpp)
target_link_libraries(talker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(talker ${catkin_EXPORTED_TARGETS})



add_executable(listener src/listener1.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener ${catkin_EXPORTED_TARGETS})



include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

