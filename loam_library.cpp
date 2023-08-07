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
const std::string default_yaml_path = "./configs/default.yaml";
const std::string default_log_path = "/data/log/oh_my_loam";
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
oh_my_loam::OhMyLoam *loam;
common::PointCloudPtr cloud;


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

    pointcloud_output = new slambench::outputs::Output("PointCloud", slambench::values::VT_POINTCLOUD, true);
    pointcloud_output->SetKeepOnlyMostRecent(true);

    track_frame_output = new slambench::outputs::Output("Tracking Frame", slambench::values::VT_FRAME);
    track_frame_output->SetKeepOnlyMostRecent(true);

    slam_settings->GetOutputManager().RegisterOutput(pose_output);
    slam_settings->GetOutputManager().RegisterOutput(pointcloud_output);
    slam_settings->GetOutputManager().RegisterOutput(track_frame_output);

    // Inspect sensors
    lidar_sensor = (slambench::io::LidarSensor*)slam_settings->get_sensors().GetSensor(slambench::io::LidarSensor::kLidarType);
    if (lidar_sensor == nullptr) {
        std::cerr << "Invalid sensors found, Lidar not found." << std::endl;
        return false;
    }

    // Start LOAM
    loam = new oh_my_loam::OhMyLoam();
    if (!loam->Init()) {
        std::cerr << "Failed to initilize slam system." << std::endl;
    }

    return true;
}


bool sb_update_frame(SLAMBenchLibraryHelper *slam_settings , slambench::io::SLAMFrame *s) {

	if(s->FrameSensor == lidar_sensor) {
		cloud = common::PointCloudPtr(new common::PointCloud);
        int num_points = s->GetSize() / (sizeof(float) * 3);
        cloud->points.resize(num_points);
        cloud->width = num_points;
        cloud->height = 1;
        cloud->is_dense = false; // If your cloud might have NaN values

        memcpy(cloud->points.data(), s->GetData(), s->GetSize());
        
        return true;
	}
	
	return false;
}