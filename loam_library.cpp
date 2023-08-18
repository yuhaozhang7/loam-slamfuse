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
#include <pcl/common/transforms.h>

#include <chrono>
#include <filesystem>
#include <functional>
#include <thread>

#include "common/common.h"
#include "oh_my_loam/oh_my_loam.h"


// const std::string default_yaml_path = "/deps/aloam/configs/default.yaml";
const std::string default_yaml_path = "/home/yuhao/loam/configs/default.yaml";

// Parameters
std::string lidar_name;
std::string yaml_path;

// Sensors
slambench::io::LidarSensor *lidar_sensor;

size_t frame_id = 0;

slambench::TimeStamp last_frame_timestamp;
double current_timestamp;

// Outputs
slambench::outputs::Output *pose_output;
slambench::outputs::Output *pointcloud_output;

// System
static oh_my_loam::OhMyLoam loam;
// contains rotation only
Eigen::Matrix4f velo_2_lgrey = (Eigen::Matrix4f() << 7.027555e-03f, -9.999753e-01f,  2.599616e-05f,  0.000000e+00f,
                                                    -2.254837e-03f, -4.184312e-05f, -9.999975e-01f,  0.000000e+00f,
                                                     9.999728e-01f,  7.027479e-03f, -2.255075e-03f,  0.000000e+00f,
                                                     0.000000e+00f,  0.000000e+00f,  0.000000e+00f,  1.000000e+00f).finished();
// input cloud
common::PointCloudPtr cloud;
// outputs of loam->Run()
std::vector<oh_my_loam::TPointCloudConstPtr> cloud_vector;
Eigen::Matrix4f pose;


bool sb_new_slam_configuration(SLAMBenchLibraryHelper * slam_settings) {

    slam_settings->addParameter(TypedParameter<std::string>("", "configuration", "path to configuration YAML file", &yaml_path, &default_yaml_path));

    return true;
}


bool sb_init_slam_system(SLAMBenchLibraryHelper *slam_settings) {
    
    // Declare Outputs
    pose_output = new slambench::outputs::Output("Pose", slambench::values::VT_POSE, true);

    pointcloud_output = new slambench::outputs::Output("PointCloud", slambench::values::VT_POINTCLOUD, true);
    pointcloud_output->SetKeepOnlyMostRecent(true);

    slam_settings->GetOutputManager().RegisterOutput(pose_output);
    slam_settings->GetOutputManager().RegisterOutput(pointcloud_output);

    // Inspect sensors
    lidar_sensor = (slambench::io::LidarSensor*)slam_settings->get_sensors().GetSensor(slambench::io::LidarSensor::kLidarType);
    if (lidar_sensor == nullptr) {
        std::cerr << "Invalid sensors found, Lidar not found." << std::endl;
        delete pose_output;
        delete pointcloud_output;
        return false;
    }

    // Start LOAM
    common::YAMLConfig::Instance()->Init(default_yaml_path);

    bool is_log_to_file = common::YAMLConfig::Instance()->Get<bool>("log_to_file");
    std::string log_path = common::YAMLConfig::Instance()->Get<std::string>("log_path");
    std::string lidar = common::YAMLConfig::Instance()->Get<std::string>("lidar");

    if (!loam.Init()) {
        std::cerr << "Failed to initialize slam system." << std::endl;
        return false;
    }

    std::cout << "A-LOAM initialized" << std::endl;

    return true;
}


bool sb_update_frame(SLAMBenchLibraryHelper *slam_settings , slambench::io::SLAMFrame *s) {

	if (s->FrameSensor == lidar_sensor) {

        last_frame_timestamp = s->Timestamp;
        current_timestamp = static_cast<double>(s->Timestamp.S) + static_cast<double>(s->Timestamp.Ns) / 1e9;

        // char* data = (char*)s->GetData();
        // uint32_t count = *(uint32_t*)data;
        // float *fdata = (float*)(data);

        float *fdata = static_cast<float*>(s->GetData());
        int count = s->GetSize()/(4 * sizeof(float));

        cloud = common::PointCloudPtr(new common::PointCloud);

        for(int i = 0; i < count; ++i) {
            float x = fdata[i*4];
            float y = fdata[i*4+1];
            float z = fdata[i*4+2];
            float r = fdata[i*4+3];
            common::Point point;
            point.x = x;
            point.y = y;
            point.z = z;
            cloud->points.push_back(point);
        }
        cloud->width = cloud->points.size();
        cloud->height = 1;
        
        return true;
	}
	
	return false;
}


bool sb_process_once(SLAMBenchLibraryHelper *slam_settings) {

    common::Pose3d pose3d;
    cloud_vector = loam.Run(current_timestamp, cloud, &pose3d);
    pose = pose3d.TransMat().cast<float>();

    return true;
}


bool sb_update_outputs(SLAMBenchLibraryHelper *lib, const slambench::TimeStamp *ts_p) {
    (void)lib;

    slambench::TimeStamp ts = *ts_p;

    if (pose_output->IsActive()) {
        std::lock_guard<FastLock> lock (lib->GetOutputManager().GetLock());
		pose_output->AddPoint(ts, new slambench::values::PoseValue(velo_2_lgrey * pose));
    }

    if (pointcloud_output->IsActive()) {
        oh_my_loam::TPointCloudConstPtr cloud_corn = cloud_vector.at(0);
        oh_my_loam::TPointCloudConstPtr cloud_surf = cloud_vector.at(1);

        oh_my_loam::TPointCloudPtr cloud_corn_trans(new oh_my_loam::TPointCloud);
        oh_my_loam::TPointCloudPtr cloud_surf_trans(new oh_my_loam::TPointCloud);
        Eigen::Matrix4f pose_map = velo_2_lgrey * pose;

        pcl::transformPointCloud(*cloud_corn, *cloud_corn_trans, pose_map);
        pcl::transformPointCloud(*cloud_surf, *cloud_surf_trans, pose_map);

        auto slambench_point_cloud = new slambench::values::PointCloudValue();
        for(const auto &p : *cloud_corn_trans) {
            slambench_point_cloud->AddPoint(slambench::values::Point3DF(p.x, p.y, p.z));
        }
        for(const auto &p : *cloud_surf_trans) {
            slambench_point_cloud->AddPoint(slambench::values::Point3DF(p.x, p.y, p.z));
        }

        // Take lock only after generating the map
        std::lock_guard<FastLock> lock (lib->GetOutputManager().GetLock());
        pointcloud_output->AddPoint(ts, slambench_point_cloud);
    }

    return true;
}


bool sb_clean_slam_system() {
    delete pose_output;
    delete pointcloud_output;
    delete lidar_sensor;
    return true;
}
