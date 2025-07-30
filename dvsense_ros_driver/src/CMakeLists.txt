# Add executable
add_executable(dvsense_ros_driver dvsense_ros_driver_node.cpp)
# Link libraries
target_link_libraries(dvsense_ros_driver 
${DvSense_LIBS}
)