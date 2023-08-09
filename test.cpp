/*

 Copyright (c) 2023 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

*/

#include <chrono>
#include <Eigen/Eigen>
#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>

#include <chrono>
#include <filesystem>
#include <functional>
// #include <thread>

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

// System
// oh_my_loam::OhMyLoam *loam;
static oh_my_loam::OhMyLoam loam;
// common::PointCloudPtr cloud;
Eigen::Matrix4f pose;


bool sb_new_slam_configuration() {
    return true;
}


bool sb_init_slam_system() {

    // Start LOAM
    common::YAMLConfig::Instance()->Init(default_yaml_path);

    bool is_log_to_file = common::YAMLConfig::Instance()->Get<bool>("log_to_file");
    std::string log_path = common::YAMLConfig::Instance()->Get<std::string>("log_path");
    std::string lidar = common::YAMLConfig::Instance()->Get<std::string>("lidar");
    // logging
    // common::InitG3Logging(is_log_to_file, "oh_my_loam_" + lidar, log_path);
    if (!loam.Init()) {
        std::cerr << "Failed to initilize slam system." << std::endl;
    }

    std::cout << "LOAM start..., lidar = " << default_lidar_name << std::endl;

    return true;
}


bool sb_update_frame() {
	return true;
}


Eigen::Matrix4f PointCloudHandler(const common::PointCloudConstPtr &cloud, oh_my_loam::OhMyLoam *const slam) {
    auto millisecs = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch());
    double timestamp = millisecs.count() / 1000.0;
    static size_t frame_id = 0;
    std::cout << "Ohmyloam: frame_id = " << ++frame_id
              << ", timestamp = " << FMT_TIMESTAMP(timestamp)
              << ", point_number = " << cloud->size() << std::endl;
    common::Pose3d pose3d;
    slam->Run(timestamp, cloud, &pose3d);
    Eigen::Matrix4f pose = pose3d.TransMat().cast<float>();
    return pose;
}


bool sb_process_once() {

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


bool sb_update_outputs() {
    return true;
}


bool sb_clean_slam_system() {
    return true;
}


int main() {
	sb_init_slam_system();
	for(int i = 0; i<150; i++) {
		sb_process_once();
	}
}
