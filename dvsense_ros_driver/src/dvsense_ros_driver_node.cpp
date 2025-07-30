// This file is part of DVSense-ROS - the NWPU DVS ROS Package

#include "../include/dvsense_ros_driver.h"

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "dvsense_ros_driver_node");

	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	DvsenseRosDriver *driver = new DvsenseRosDriver(nh);

    driver->AsynchronousCatch();
	ros::waitForShutdown();
	driver->Close();
	return 0;
}