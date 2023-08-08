/*

 Copyright (c) 2023 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

*/

#include <Parameters.h>
#include <SLAMBenchAPI.h>
#include <io/SLAMFrame.h>
#include <io/sensor/LidarSensor.h>
#include <io/sensor/GroundTruthSensor.h>
#include <chrono>
#include <Eigen/Eigen>
#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>

#include <chrono>
#include <filesystem>
#include <functional>
#include <thread>

#include "common/common.h"
#include "oh_my_loam/oh_my_loam.h"

// Default Parameters
const std::string default_lidar_name = "VLP";
const std::string default_yaml_path = "/home/yuhao/loam/configs/default.yaml";
const std::string default_log_path = "/data/log/oh_my_loam";
const std::string dirname = "/mnt/d/Download/Dataset/Kitti/2011_09_26/2011_09_26_drive_0005_sync/velodyne_points/pcd/";
static int lidar_cout = 0;
const bool default_log_to_file = false;

// Parameters
std::string lidar_name;
std::string yaml_path;
std::string log_path;
bool log_to_file;

// Sensors
slambench::io::LidarSensor *lidar_sensor;

// Outputs
slambench::outputs::Output *pose_output;
slambench::outputs::Output *pointcloud_output;
slambench::outputs::Output *track_frame_output;

// System
// oh_my_loam::OhMyLoam *loam;
static oh_my_loam::OhMyLoam loam;
// common::PointCloudPtr cloud;
Eigen::Matrix4f pose;


bool sb_new_slam_configuration(SLAMBenchLibraryHelper * slam_settings) {

    slam_settings->addParameter(TypedParameter<std::string>("", "lidar-name", "name of lidar sensor", &lidar_name, &default_lidar_name));
    slam_settings->addParameter(TypedParameter<std::string>("", "configuration", "path to configuration YAML file", &yaml_path, &default_yaml_path));
    slam_settings->addParameter(TypedParameter<std::string>("", "log-path", "path to log file", &log_path, &default_log_path));
    slam_settings->addParameter(TypedParameter<bool>("", "log-to-file", "log to file or not", &log_to_file, &default_log_to_file));

    return true;
}


bool sb_init_slam_system(SLAMBenchLibraryHelper *slam_settings) {
    
    // Declare Outputs
    pose_output = new slambench::outputs::Output("Pose", slambench::values::VT_POSE, true);

    // pointcloud_output = new slambench::outputs::Output("PointCloud", slambench::values::VT_POINTCLOUD, true);
    // pointcloud_output->SetKeepOnlyMostRecent(true);

    // track_frame_output = new slambench::outputs::Output("Tracking Frame", slambench::values::VT_FRAME);
    // track_frame_output->SetKeepOnlyMostRecent(true);

    slam_settings->GetOutputManager().RegisterOutput(pose_output);
    // slam_settings->GetOutputManager().RegisterOutput(pointcloud_output);
    // slam_settings->GetOutputManager().RegisterOutput(track_frame_output);

    // Inspect sensors
    lidar_sensor = (slambench::io::LidarSensor*)slam_settings->get_sensors().GetSensor(slambench::io::LidarSensor::kLidarType);
    if (lidar_sensor == nullptr) {
        std::cerr << "Invalid sensors found, Lidar not found." << std::endl;
        return false;
    }

    // Start LOAM
    common::YAMLConfig::Instance()->Init(default_yaml_path);

    bool is_log_to_file = common::YAMLConfig::Instance()->Get<bool>("log_to_file");
    std::string log_path = common::YAMLConfig::Instance()->Get<std::string>("log_path");
    std::string lidar = common::YAMLConfig::Instance()->Get<std::string>("lidar");
    // logging
    common::InitG3Logging(is_log_to_file, "oh_my_loam_" + lidar, log_path);
    if (!loam.Init()) {
        std::cerr << "Failed to initilize slam system." << std::endl;
    }

    std::cout << "LOAM start..., lidar = " << default_lidar_name << std::endl;

    return true;
}


bool sb_update_frame(SLAMBenchLibraryHelper *slam_settings , slambench::io::SLAMFrame *s) {

	if (s->FrameSensor == lidar_sensor) {
        /*
        cloud = common::PointCloudPtr(new common::PointCloud);
        
        int num_points = s->GetSize() / (sizeof(float) * 3);
        cloud->points.resize(num_points);
        cloud->width = num_points;
        cloud->height = 1;
        cloud->is_dense = false; // If your cloud might have NaN values
        
        memcpy(cloud->points.data(), s->GetData(), s->GetSize());
        */
        
        /*
        cloud = common::PointCloudPtr(new common::PointCloud);
        std::stringstream tmp_filename;
        tmp_filename << std::setw(10) << std::setfill('0') << lidar_cout;
        std::string lidar_file_pcd = tmp_filename.str() + ".pcd";
        lidar_file_pcd = dirname + lidar_file_pcd;
        lidar_cout++;

        pcl::io::loadPCDFile(lidar_file_pcd, *cloud);
        */
        
        
        return true;
	}
	
	return false;
}


Eigen::Matrix4f PointCloudHandler(const common::PointCloudConstPtr &cloud, oh_my_loam::OhMyLoam *const slam) {
    auto millisecs = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch());
    double timestamp = millisecs.count() / 1000.0;
    static size_t frame_id = 0;
    AINFO << "Ohmyloam: frame_id = " << ++frame_id
            << ", timestamp = " << FMT_TIMESTAMP(timestamp)
            << ", point_number = " << cloud->size();
    common::Pose3d pose3d;
    slam->Run(timestamp, cloud, &pose3d);
    Eigen::Matrix4f pose = pose3d.TransMat().cast<float>();
    return pose;
}


bool sb_process_once(SLAMBenchLibraryHelper *slam_settings) {

    common::PointCloudPtr cloud(new common::PointCloud);
    std::stringstream tmp_filename;
    tmp_filename << std::setw(10) << std::setfill('0') << lidar_cout;
    std::string lidar_file_pcd = tmp_filename.str() + ".pcd";
    lidar_file_pcd = dirname + lidar_file_pcd;
    std::cout << lidar_file_pcd << std::endl;
    lidar_cout++;

    pcl::io::loadPCDFile(lidar_file_pcd, *cloud);

    pose = PointCloudHandler(cloud, &loam);
    std::cout << pose << std::endl;

    // std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 10 Hz

    return true;
}


bool sb_update_outputs(SLAMBenchLibraryHelper *lib, const slambench::TimeStamp *ts_p) {
    (void)lib;

    slambench::TimeStamp ts = *ts_p;

    if(pose_output->IsActive()) {
        std::lock_guard<FastLock> lock (lib->GetOutputManager().GetLock());
		pose_output->AddPoint(ts, new slambench::values::PoseValue(pose));
    }

    return true;
}


bool sb_clean_slam_system() {
    delete pose_output;
    delete lidar_sensor;
    return true;
}
