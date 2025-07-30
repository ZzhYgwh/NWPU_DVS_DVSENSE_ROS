/// refer at https://sdk.dvsense.com/zh/html/camera_zh.html

#include <DvsCameraManager.hpp>

#include <iostream>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>

#include <thread>
#include <mutex>

#include <dvs_msgs/EventArray.h>
#include <sensor_msgs/CameraInfo.h>
#include <vector>
#include <deque>

#include <condition_variable>
#include <atomic>

class DvsenseRosDriver
{
public:
    DvsenseRosDriver(ros::NodeHandle & nh)
    {
        this->nh = nh;
        std::string config_file;

        nh.getParam("/dvsense_ros_driver/config_file", config_file);
        // config_file = "/home/hao/platform_ws/src/nwpu_dvsense_ros/dvsense_ros_driver/config/DvsLume.yaml";
        ROS_INFO_STREAM("config_file = " << config_file);
        // ROS_ERROR_STREAM("find all cameras ");
        // find all cameras
        dvsense::DvsCameraManager cameraManager;
        cameraDescs = cameraManager.getCameraDescs();

        printConfig(config_file);

        ROS_INFO_STREAM("Parse DVSenseYaml = " << config_file);
        // todo: 默认多个camera 使用同一套参数
        if(!ParseDVSenseYaml(config_file))
        {
            ROS_ERROR_STREAM("Failed to Parse DVSenseYaml");
        }

        ROS_WARN_STREAM("SetParam");
        camera_ptr_vec.reserve(cameraDescs.size());
        for(auto& cam: cameraDescs)
        {
            camera_ptr_vec.push_back(cameraManager.openCamera(cam.serial));
            SetParam(camera_ptr_vec.back(), true);
        }

        std::string topic_prefix = "/dvsense/event_";
        camera_pub_vec.resize(cameraDescs.size());  // 让vector真正有size
        int pub_id = 0;
        for (auto& pub : camera_pub_vec)
        {
            pub = nh.advertise<dvs_msgs::EventArray>(topic_prefix + std::to_string(pub_id++), 10);
        }

        topic_prefix = "/dvsense/camera_info_";
        camera_info_pub_vec.resize(cameraDescs.size());  // 让vector真正有size
        pub_id = 0;
        for (auto& pub : camera_info_pub_vec)
        {
            pub = nh.advertise<sensor_msgs::CameraInfo>(topic_prefix + std::to_string(pub_id++), 10);
        }

        assert(cam_info_msg_vec.size() == camera_info_pub_vec.size() && "event paramater size is not consistent with parameter publish");
    }

    ~DvsenseRosDriver(){};

    // debug: get device list()
    void INFO_DeviceList()
    {
        ROS_INFO_STREAM("INFO_DeviceList");
        for(auto& desc: cameraDescs)
        {
            ROS_INFO_STREAM("desc.serial = " << desc.serial);
        }
    }

    void AsynchronousCatch()
    {
        if(output_type == "ros")
        {
            ROS_INFO_STREAM("AsynchronousCatch");
            int file_name_idex = 0;
            ros::Time start_time = ros::Time::now();
            for(auto cam: camera_ptr_vec)
            {
                cam->start();
                int pub_events_rate_local = pub_events_rate;
                int FRAME_EVENT_NUM_THRESHOLD_SET = FRAME_EVENT_NUM_THRESHOLD;
                std::string pub_type_set = pub_type;

                // static std::vector<dvs_msgs::Event> event_buffer;
                // static int event_counter = 0;  
               
                ROS_INFO_STREAM("set C");
                // 启动数据转播
                event_thread_vec.emplace_back([this, file_name_idex, start_time, pub_type_set, FRAME_EVENT_NUM_THRESHOLD_SET]() {
                std::vector<dvs_msgs::Event> event_buffer;
                long int event_counter = 0;
                // ROS_INFO_STREAM("Initialize EventsStream Publish");
                running.store(true);
                while (running.load() && ros::ok()) {
                    // ROS_INFO_STREAM("get in EventsStream Publish");
                    std::vector<std::pair<const dvsense::Event2D*, const dvsense::Event2D*>> event_array_pointer_vec_copy;

                    {
                        std::lock_guard<std::mutex> lock(event_mutex);
                        if(event_array_pointer_vec.empty())
                            continue;
                        event_array_pointer_vec_copy.swap(event_array_pointer_vec);  // 交换提高效率
                    }
                    // ROS_INFO_STREAM("event_array_pointer_vec_copy.size() = " << event_array_pointer_vec_copy.size());
                    // 遍历拷贝数据
                    for (const auto& event_data_pair : event_array_pointer_vec_copy) {
                        for (const dvsense::Event2D* it = event_data_pair.first; it != event_data_pair.second; ++it) {
                            dvs_msgs::Event event;
                            event.x = it->x;
                            event.y = it->y;
                            event.polarity = it->polarity;
                            event.ts = start_time + ros::Duration(it->timestamp * 1e-6);

                            event_buffer.push_back(event);
                            ++event_counter;

                            if (pub_type_set == "frame" && event_counter >= FRAME_EVENT_NUM_THRESHOLD_SET) {
                                dvs_msgs::EventArray event_msg;
                                event_msg.header.stamp = event_buffer.back().ts;
                                event_msg.header.frame_id = "event_" + std::to_string(file_name_idex);
                                event_msg.height = cam_info_msg_vec[file_name_idex].height;
                                event_msg.width  = cam_info_msg_vec[file_name_idex].width;
                                event_msg.events = std::move(event_buffer);

                                camera_pub_vec[file_name_idex].publish(event_msg);

                                cam_info_msg_vec[file_name_idex].header = event_msg.header;
                                camera_info_pub_vec[file_name_idex].publish(cam_info_msg_vec[file_name_idex]);

                                event_buffer.clear();
                                event_counter = 0;
                            }
                        }
                        }
                    }
                    // ROS_INFO_STREAM("Exist EventsStream Publish");
                });

                // ROS_INFO_STREAM("set A");

                // 启动线程监听事件流
                event_thread_vec.emplace_back([this, cam, file_name_idex, start_time]() {
                uint32_t cb_id = cam->addEventsStreamHandleCallback(
                    [this, file_name_idex, start_time](const dvsense::Event2D* begin, const dvsense::Event2D* end) {
                            {
                                // ROS_INFO_STREAM("get in EventsStreamHandleCallback");
                                std::lock_guard<std::mutex> lock(event_mutex);
                                event_array_pointer_vec.emplace_back(begin, end);
                            }
                            // ROS_INFO_STREAM("event_array_pointer_vec.size = " << event_array_pointer_vec.size());
                        });
                
                    // 保存回调ID（线程安全）
                    // {
                    //     std::lock_guard<std::mutex> lock(callback_mutex);
                    //     event_callback_id_vec.push_back(cb_id);
                    // }
                });
                // ROS_INFO_STREAM("set B");
                // event_thread_vec.back().detach();


                ROS_INFO_STREAM("publish event " << std::to_string(file_name_idex) << " ... ");
                file_name_idex++;
            }
        }
        else
        {
            int file_name_idex = 0;
            for(auto cam: camera_ptr_vec)
            {
                cam->start();
                cam->startRecording(out_file_path + std::to_string(file_name_idex));  // 开始录制
                ROS_INFO_STREAM("record camera " << std::to_string(file_name_idex++) << " ... ");
            }
        }

    }

    // todo: 暂不支持
    void SynchronousCatch()
    {
        // camera->setBatchEventsNum(1000); // 设置获取1000个事件

        // dvsense::Event2DVector events;
        // bool ret = camera->getNextBatch(events);
        // for (auto event =events) {
        //     // Do something with the events
        // }
    }


    void Close()
    {
        if(output_type == "ros")
        {
            running.store(false);
            // **等待所有线程执行完毕**
            for (auto& t : event_thread_vec)
            {
                if (t.joinable())
                {
                    t.join();
                }
            }


            int file_name_idex = 0;
            for(auto cam: camera_ptr_vec)
            {
                {
                    std::lock_guard<std::mutex> lock(callback_mutex);
                    cam->removeEventsStreamHandleCallback(event_callback_id_vec[file_name_idex]);
                }
                ROS_INFO_STREAM("close camera " << std::to_string(file_name_idex) << " ... ");
                file_name_idex++;
                cam->stop();
            }
        }
        else
        {
            int file_name_idex = 0;
            for(auto cam: camera_ptr_vec)
            {
                cam->stopRecording();  // 开始录制
                ROS_INFO_STREAM("close camera " << std::to_string(file_name_idex++) << " ... ");
                cam->stop();
            }
        }
        
    }

private:

    void printConfig(const std::string& file_path) {
        // YAML::Node config = YAML::LoadFile(file_path);
        // YAML::Node DvsLumeNode = config["DvsLume"];

        YAML::Node DvsLumeNode = YAML::LoadFile(file_path);

        if (!DvsLumeNode) {
            ROS_ERROR_STREAM("Error: 'DvsLume' node not found in YAML file!");
            return;
        }

        ROS_INFO_STREAM("===== DvsLume Configuration =====");

        if (DvsLumeNode["Bias"]) {
            ROS_INFO_STREAM("Bias:");
            ROS_INFO_STREAM("  bias_diff_on: " << DvsLumeNode["Bias"]["bias_diff_on"].as<int>());
            ROS_INFO_STREAM("  bias_diff_off: " << DvsLumeNode["Bias"]["bias_diff_off"].as<int>());
            ROS_INFO_STREAM("  bias_fo: " << DvsLumeNode["Bias"]["bias_fo"].as<int>());
            ROS_INFO_STREAM("  bias_hpf: " << DvsLumeNode["Bias"]["bias_hpf"].as<int>());
            ROS_INFO_STREAM("  bias_refr: " << DvsLumeNode["Bias"]["bias_refr"].as<int>());
        }

        if (DvsLumeNode["ROI"]) {
            ROS_INFO_STREAM("ROI:");
            ROS_INFO_STREAM("  x: " << DvsLumeNode["ROI"]["x_"].as<int>());
            ROS_INFO_STREAM("  y: " << DvsLumeNode["ROI"]["y_"].as<int>());
            ROS_INFO_STREAM("  width: " << DvsLumeNode["ROI"]["width"].as<int>());
            ROS_INFO_STREAM("  height: " << DvsLumeNode["ROI"]["height"].as<int>());
            ROS_INFO_STREAM("  rescale: " << DvsLumeNode["ROI"]["rescale"].as<float>());
        }

        if (DvsLumeNode["Anti_Flicker"]) {
            ROS_INFO_STREAM("Anti_Flicker:");
            ROS_INFO_STREAM("  enable: " << DvsLumeNode["Anti_Flicker"]["enable"].as<bool>());
            ROS_INFO_STREAM("  low_frequency: " << DvsLumeNode["Anti_Flicker"]["low_frequency"].as<int>());
            ROS_INFO_STREAM("  high_frequency: " << DvsLumeNode["Anti_Flicker"]["high_frequency"].as<int>());
            ROS_INFO_STREAM("  fliter_mode: " << DvsLumeNode["Anti_Flicker"]["fliter_mode"].as<std::string>());
            ROS_INFO_STREAM("  duty_cycle: " << DvsLumeNode["Anti_Flicker"]["duty_cycle"].as<float>());
            ROS_INFO_STREAM("  start_threshold: " << DvsLumeNode["Anti_Flicker"]["start_threshold"].as<int>());
            ROS_INFO_STREAM("  stop_threshold: " << DvsLumeNode["Anti_Flicker"]["stop_threshold"].as<int>());
        }

        if (DvsLumeNode["Event_Trail_Filter"]) {
            ROS_INFO_STREAM("Event_Trail_Filter:");
            ROS_INFO_STREAM("  enable: " << DvsLumeNode["Event_Trail_Filter"]["enable"].as<bool>());
            ROS_INFO_STREAM("  threshold: " << DvsLumeNode["Event_Trail_Filter"]["threshold"].as<int>());
            ROS_INFO_STREAM("  type: " << DvsLumeNode["Event_Trail_Filter"]["type"].as<std::string>());
        }

        if (DvsLumeNode["Event_Rate_Controller"]) {
            ROS_INFO_STREAM("Event_Rate_Controller:");
            ROS_INFO_STREAM("  enable: " << DvsLumeNode["Event_Rate_Controller"]["enable"].as<bool>());
            ROS_INFO_STREAM("  max_event_rate: " << DvsLumeNode["Event_Rate_Controller"]["max_event_rate"].as<int>());
        }

        if (DvsLumeNode["trriger_enable"]) {
            ROS_INFO_STREAM("Trigger Enable: " << DvsLumeNode["trriger_enable"].as<bool>());
        }

        if (DvsLumeNode["Output_Type"]) {
            ROS_INFO_STREAM("Output Type: " << DvsLumeNode["Output_Type"].as<std::string>());
        }

        ROS_INFO_STREAM("=================================");
    }

    // 改成模板支持 boost::array、std::array、裸数组等
    template <typename ArrayT>
    void fill_array(const YAML::Node& node, ArrayT& target, int size)
    {
        std::vector<double> data = node.as<std::vector<double>>();
        if (data.size() != size) throw std::runtime_error("Wrong size in fill_array");
        for (int i = 0; i < size; ++i)
            target[i] = data[i];
    }


    bool ParseEventYaml(const std::string& file_path)
    {
        YAML::Node event_params_;
        try {
            event_params_ = YAML::LoadFile(file_path);
        } catch (const YAML::Exception& e) {
            ROS_ERROR_STREAM("YAML load error: " << e.what());
            return false;
        }

        int event_num = event_params_["num"].as<int>();
        for (int idx = 0; idx < event_num; ++idx)
        {
            std::string key = "event" + std::to_string(idx);
            if (!event_params_[key]) {
                ROS_ERROR_STREAM("Missing key: " << key);
                continue;
            }

            const YAML::Node& cam_node = event_params_[key];
            sensor_msgs::CameraInfo info;

            info.width = cam_node["width"].as<int>();
            info.height = cam_node["height"].as<int>();
            info.distortion_model = cam_node["distortion_model"].as<std::string>();

            info.D = cam_node["D"].as<std::vector<double>>();

            fill_array(cam_node["K"], info.K, 9);
            fill_array(cam_node["R"], info.R, 9);
            fill_array(cam_node["P"], info.P, 12);

            cam_info_msg_vec.push_back(info);
        }

        return true;
    }

    bool ParseDVSenseYaml(const std::string& file_path) 
    {

        try
        {
            // YAML::Node config = YAML::Load(file_path);
            // YAML::Node DvsLumeNode = config["DvsLume"];

            YAML::Node DvsLumeNode = YAML::LoadFile(file_path);
            // YAML::Node DvsLumeNode = config["DvsLume"];
            // YAML::Node DvsLumeNode = config["DvsLume"];
            // ROS_INFO_STREAM("DvsLumeNode: " << DvsLumeNode);

            if (DvsLumeNode["Bias"] && DvsLumeNode["Bias"].IsMap())
            {
                bias_diff_on = DvsLumeNode["Bias"]["bias_diff_on"].as<int>();
                bias_diff_off = DvsLumeNode["Bias"]["bias_diff_off"].as<int>();
                bias_fo = DvsLumeNode["Bias"]["bias_fo"].as<int>();
                bias_hpf = DvsLumeNode["Bias"]["bias_hpf"].as<int>();
                bias_refr = DvsLumeNode["Bias"]["bias_refr"].as<int>();
            }
            else
            {
                ROS_ERROR_STREAM("missing DvsLumeNode[\"Bias\"]");
            }

            if (DvsLumeNode["ROI"] && DvsLumeNode["ROI"].IsMap())
            {
                roi_x = DvsLumeNode["ROI"]["x_"].as<int>();
                roi_y = DvsLumeNode["ROI"]["y_"].as<int>();
                width = DvsLumeNode["ROI"]["width"].as<int>();
                height = DvsLumeNode["ROI"]["height"].as<int>();
                rescale = DvsLumeNode["ROI"]["rescale"].as<float>();       
            }
            else
            {
                ROS_ERROR_STREAM("missing DvsLumeNode[\"ROI\"]");
            }

            if (DvsLumeNode["Anti_Flicker"] && DvsLumeNode["Anti_Flicker"].IsMap())
            {
                anti_flicker_enable = DvsLumeNode["Anti_Flicker"]["enable"].as<bool>();
                low_frequency = DvsLumeNode["Anti_Flicker"]["low_frequency"].as<int>();
                high_frequency = DvsLumeNode["Anti_Flicker"]["high_frequency"].as<int>();
                fliter_mode = DvsLumeNode["Anti_Flicker"]["fliter_mode"].as<std::string>();
                duty_cycle = DvsLumeNode["Anti_Flicker"]["duty_cycle"].as<float>();
                start_threshold = DvsLumeNode["Anti_Flicker"]["start_threshold"].as<int>();
                stop_threshold = DvsLumeNode["Anti_Flicker"]["stop_threshold"].as<int>();
            }
            else
            {
                ROS_ERROR_STREAM("missing DvsLumeNode[\"Anti_Flicker\"]");
            }


            if (DvsLumeNode["Event_Trail_Filter"] && DvsLumeNode["Event_Trail_Filter"].IsMap())
            {
                trail_filter_enable = DvsLumeNode["Event_Trail_Filter"]["enable"].as<bool>();
                threshold = DvsLumeNode["Event_Trail_Filter"]["threshold"].as<int>();
                trail_filter_type = DvsLumeNode["Event_Trail_Filter"]["type"].as<std::string>();
            }
            else
            {
                ROS_ERROR_STREAM("missing DvsLumeNode[\"Event_Trail_Filter\"]");
            }


            if (DvsLumeNode["Event_Rate_Controller"] && DvsLumeNode["Event_Rate_Controller"].IsMap())
            {
                rate_cotroller_enable = DvsLumeNode["Event_Rate_Controller"]["enable"].as<bool>();
                max_event_rate = DvsLumeNode["Event_Rate_Controller"]["max_event_rate"].as<int>();
            }            
            else
            {
                ROS_ERROR_STREAM("missing DvsLumeNode[\"Event_Rate_Controller\"]");
            }

            trigger_enable = DvsLumeNode["trriger_enable"].as<bool>();
            output_type = DvsLumeNode["Output_Type"].as<std::string>();

            if(!ParseEventYaml(DvsLumeNode["Camera_Info_File"].as<std::string>()))
            {
                ROS_ERROR_STREAM("missing Event Yaml");
                return false;
            }

            if(DvsLumeNode["pub_type"] && DvsLumeNode["FRAME_EVENT_NUM_THRESHOLD"])
            pub_type = DvsLumeNode["pub_type"].as<std::string>();
            ROS_ERROR_STREAM("pub_type: " << pub_type);
            FRAME_EVENT_NUM_THRESHOLD = DvsLumeNode["FRAME_EVENT_NUM_THRESHOLD"].as<int>();
            ROS_ERROR_STREAM("FRAME_EVENT_NUM_THRESHOLD: " << FRAME_EVENT_NUM_THRESHOLD);

            return true;

        }
        catch(const std::exception& e)
        {
            ROS_ERROR_STREAM(e.what() << '\n');
            return false;
        } 
    }

    bool SetParam(dvsense::CameraDevice& device_, bool verbose = false)
    {
        try
        {
            bool success = true;

            // Bias Tool
            std::shared_ptr<dvsense::CameraTool> bias = device_->getTool(dvsense::ToolType::TOOL_BIAS);
            if (!bias) {
                ROS_ERROR_STREAM("Failed to get Bias Tool.");
                return false;
            }
            success &= bias->setParam("bias_diff_on", bias_diff_on);
            success &= bias->setParam("bias_diff_off", bias_diff_off);
            success &= bias->setParam("bias_fo", bias_fo);
            success &= bias->setParam("bias_hpf", bias_hpf);
            success &= bias->setParam("bias_refr", bias_refr);

            // ROI Tool
            std::shared_ptr<dvsense::CameraTool> roi_tool = device_->getTool(dvsense::ToolType::TOOL_ROI);
            if (!roi_tool) {
                ROS_ERROR_STREAM("Failed to get ROI Tool.");
                return false;
            }
            success &= roi_tool->setParam("x", roi_x);
            success &= roi_tool->setParam("y", roi_y);
            success &= roi_tool->setParam("width", width);
            success &= roi_tool->setParam("height", height);

            // Anti-Flicker Tool
            std::shared_ptr<dvsense::CameraTool> afk_tool = device_->getTool(dvsense::ToolType::TOOL_ANTI_FLICKER);
            if (!afk_tool) {
                ROS_ERROR_STREAM("Failed to get Anti-Flicker Tool.");
                return false;
            }
            success &= afk_tool->setParam("low_frequency", low_frequency);
            success &= afk_tool->setParam("high_frequency", high_frequency);
            success &= afk_tool->setParam("fliter_mode", fliter_mode);
            // ROS_INFO_STREAM("duty_cycle = " << duty_cycle);
            success &= afk_tool->setParam("duty_cycle", duty_cycle);
            success &= afk_tool->setParam("start_threshold", start_threshold);
            success &= afk_tool->setParam("stop_threshold", stop_threshold);
            success &= afk_tool->setParam("enable", anti_flicker_enable);

            // Event Trail Filter Tool
            std::shared_ptr<dvsense::CameraTool> etf_tool = device_->getTool(dvsense::ToolType::TOOL_EVENT_TRAIL_FILTER);
            if (!etf_tool) {
                ROS_ERROR_STREAM("Failed to get Event Trail Filter Tool.");
                return false;
            }
            success &= etf_tool->setParam("threshold", threshold);
            success &= etf_tool->setParam("type", trail_filter_type);
            success &= etf_tool->setParam("enable", trail_filter_enable);

            // Event Rate Control Tool
            std::shared_ptr<dvsense::CameraTool> erc_tool = device_->getTool(dvsense::ToolType::TOOL_EVENT_RATE_CONTROL);
            if (!erc_tool) {
                ROS_ERROR_STREAM("Failed to get Event Rate Control Tool.");
                return false;
            }
            success &= erc_tool->setParam("max_event_rate", max_event_rate);
            success &= erc_tool->setParam("enable", rate_cotroller_enable);

            // Trigger In Tool
            std::shared_ptr<dvsense::CameraTool> trigger_in_tool = device_->getTool(dvsense::ToolType::TOOL_TRIGGER_IN);
            if (!trigger_in_tool) {
                ROS_ERROR_STREAM("Failed to get Trigger In Tool.");
                return false;
            }
            success &= trigger_in_tool->setParam("enable", trigger_enable);
            // success &= trigger_in_tool->setParam("output_Type", output_type);

            // 读取参数（verbose模式）
            if (verbose)
            {
                int value = 0;
                ROS_INFO_STREAM("check camera parameters");
                // Bias Tool 参数
                success &= bias->getParam("bias_diff_on", value);
                ROS_INFO_STREAM("bias_diff_on: " << value);
                success &= bias->getParam("bias_diff_off", value);
                ROS_INFO_STREAM("bias_diff_off: " << value);
                success &= bias->getParam("bias_fo", value);
                ROS_INFO_STREAM("bias_fo: " << value);
                success &= bias->getParam("bias_hpf", value);
                ROS_INFO_STREAM("bias_hpf: " << value);
                success &= bias->getParam("bias_refr", value);
                ROS_INFO_STREAM("bias_refr: " << value);

                // ROI Tool 参数
                success &= roi_tool->getParam("x", value);
                ROS_INFO_STREAM("roi_x: " << value);
                success &= roi_tool->getParam("y", value);
                ROS_INFO_STREAM("roi_y: " << value);
                success &= roi_tool->getParam("width", value);
                ROS_INFO_STREAM("width: " << value);
                success &= roi_tool->getParam("height", value);
                ROS_INFO_STREAM("height: " << value);

                // Anti-Flicker Tool 参数
                success &= afk_tool->getParam("low_frequency", value);
                ROS_INFO_STREAM("low_frequency: " << value);
                success &= afk_tool->getParam("high_frequency", value);
                ROS_INFO_STREAM("high_frequency: " << value);
                std::string value_str = "null";
                float value_f = 0;
                success &= afk_tool->getParam("fliter_mode", value_str);
                ROS_INFO_STREAM("fliter_mode: " << value_str);
                success &= afk_tool->getParam("duty_cycle", value_f);
                ROS_INFO_STREAM("duty_cycle: " << value_f);
                success &= afk_tool->getParam("start_threshold", value);
                ROS_INFO_STREAM("start_threshold: " << value);
                success &= afk_tool->getParam("stop_threshold", value);
                ROS_INFO_STREAM("stop_threshold: " << value);
                bool value_bool = false;
                success &= afk_tool->getParam("enable", value_bool);
                ROS_INFO_STREAM("anti_flicker_enable: " << ((value_bool)?"True":"False"));

                // Event Trail Filter Tool 参数
                success &= etf_tool->getParam("threshold", value);
                ROS_INFO_STREAM("threshold: " << value);
                success &= etf_tool->getParam("type", value_str);
                ROS_INFO_STREAM("trail_filter_type: " << value_str);
                success &= etf_tool->getParam("enable", value_bool);
                ROS_INFO_STREAM("trail_filter_enable: " << ((value_bool)?"True":"False"));

                // Event Rate Control Tool 参数
                success &= erc_tool->getParam("max_event_rate", value);
                ROS_INFO_STREAM("max_event_rate: " << value);
                success &= erc_tool->getParam("enable", value_bool);
                ROS_INFO_STREAM("rate_cotroller_enable: " << ((value_bool)?"True":"False"));

                // Trigger In Tool 参数
                success &= trigger_in_tool->getParam("enable", value_bool);
                ROS_INFO_STREAM("trigger_enable: " << value_bool);
                // success &= trigger_in_tool->getParam("output_Type", value_str);
                ROS_INFO_STREAM("output_Type: " << value_str);
            }


            return success;
        }
        catch (const std::exception& e)
        {
            ROS_ERROR_STREAM("Exception caught: " << e.what());
            return false;
        }
    }

private:
    int bias_diff_on=0;
    int bias_diff_off=0;
    int bias_fo=0;
    int bias_hpf=0;
    int bias_refr=0;

    int roi_x=0;
    int roi_y=0;
    int width=1280;
    int height=720;
    float rescale=1.0;

    bool anti_flicker_enable=false;
    int low_frequency=50;
    int high_frequency=520;
    std::string fliter_mode="Band cut";
    float duty_cycle=50;
    int start_threshold=6;
    int stop_threshold=4;

    bool trail_filter_enable=false;
    int threshold=10;
    std::string trail_filter_type="TRAIL";
    
    bool rate_cotroller_enable=false;
    int max_event_rate=320;

    int pub_events_rate = 30;

    bool trigger_enable=false;
    std::string output_type="ros"; // 0 for "ros", 1 for "raw"
    std::string out_file_path = "/home/hao/Desktop/events/";

    ros::NodeHandle nh;

    // camera device list
    std::vector<dvsense::CameraDevice> camera_ptr_vec;
    std::vector<ros::Publisher> camera_pub_vec;
    std::vector<uint32_t> event_callback_id_vec;
    std::vector<std::thread> event_thread_vec;
    std::mutex callback_mutex;

    std::vector<dvsense::CameraDescription> cameraDescs;

    std::atomic<bool> running;  // 控制线程状态
    
    std::vector<sensor_msgs::CameraInfo> cam_info_msg_vec; 
    std::vector<ros::Publisher> camera_info_pub_vec;

    // pub
    std::string pub_type = "frame";
    int FRAME_EVENT_NUM_THRESHOLD = 1000;

    std::mutex event_mutex;
     std::vector<std::pair<const dvsense::Event2D*, const dvsense::Event2D*>> event_array_pointer_vec;

};