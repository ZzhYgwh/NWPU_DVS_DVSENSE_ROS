cmake_minimum_required(VERSION 3.10)
 
# Set the project name
project(dvsense_ros_driver)
 
# Set the C++ standard to use
set(CMAKE_CXX_STANDARD 20)
add_definitions(-D NOMINMAX)


# find_package(DvsenseDriver COMPONENTS Driver REQUIRED)
find_package(yaml-cpp REQUIRED)
# find_package(dvs_msgs REQUIRED)

find_package(catkin REQUIRED COMPONENTS 
roscpp
rospy
std_msgs
sensor_msgs
dvs_msgs
)
catkin_package(
    CATKIN_DEPENDS dvs_msgs sensor_msgs std_msgs
)
 
set(DvsenseDriver_INCLUDE_DIRS "/usr/include/DvsenseDriver/camera" )
# Include directories
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${DvsenseDriver_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
)

set(DvSense_LIBS
    ${catkin_LIBRARIES}
    # DvsenseDriver::Driver
    /usr/lib/libDvsenseDriver.so
    /usr/lib/libDvsenseBase.so
    /usr/lib/libDvsenseHal.so
    yaml-cpp
    pthread
)

add_subdirectory(src)
add_subdirectory(test)
