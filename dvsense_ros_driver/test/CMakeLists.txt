# Add executable
add_executable(test_devices test_devices.cpp)
# Link libraries
target_link_libraries(test_devices 
${DvSense_LIBS}
)