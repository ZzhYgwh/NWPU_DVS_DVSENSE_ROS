// This file is part of DVSense-ROS - the NWPU DVS ROS Package

#include "../include/dvsense_ros_driver.h"

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "dvsense_ros_driver_node");

	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");

	// find all cameras
	dvsense::DvsCameraManager cameraManager;
	std::vector<dvsense::CameraDescription> cameraDescs = cameraManager.getCameraDescs();
	ROS_INFO_STREAM( "INFO_DeviceList find " << cameraDescs.size() );
	for(auto& desc: cameraDescs)
	{
		ROS_INFO_STREAM( "desc.serial = " << desc.serial);
	}
	std::string config_file = "/home/hao/platform_ws/src/nwpu_dvsense_ros/dvsense_ros_driver/config/DvsLume.yaml";
        

	// auto cam_num = cameraManager.findCameras();
	// ROS_INFO_STREAM( "Camera find " << cam_num );
		
	std::vector<dvsense::CameraDevice> camera_ptr_vec;	
	camera_ptr_vec.reserve(cameraDescs.size());
	for(auto& cam: cameraDescs)
	{
		camera_ptr_vec.push_back(cameraManager.openCamera(cam.serial));
		// SetParam(camera_ptr_vec.back(), true);
	}
	ROS_INFO_STREAM( "Open Camera find " << camera_ptr_vec.size());

	dvsense::CameraDevice cam_ = camera_ptr_vec.back();

	// std::map<std::string, BasicParameterInfo> param_list = cam_->getTool()->getAllParamInfo();

	// DvsenseRosDriver *driver = new DvsenseRosDriver(nh);

    // driver->INFO_DeviceList();

    // driver->AsynchronousCatch();
	// ros::waitForShutdown();
	// driver->Close();
	return 0;
}