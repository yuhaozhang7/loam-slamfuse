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


const std::string default_yaml_path = "/deps/loam/configs/multi-thread.yaml";

// Parameters
std::string lidar_name;
std::string yaml_path;

// Sensors
slambench::io::LidarSensor *lidar_sensor;

size_t frame_id = 0;

slambench::TimeStamp last_frame_timestamp;
double current_timestamp;

// Outputs
slambench::outputs::Output *loam_pose_output;
slambench::outputs::Output *loam_pointcloud_output;

// System
static oh_my_loam::OhMyLoam loam;

std::string dataset_name;
bool show_point_cloud;
bool show_full_map;
int point_cloud_ratio;

// contains rotation only
Eigen::Matrix4f align_mat = (Eigen::Matrix4f() <<  0.0, -1.0,  0.0,  0.0,
                                                   0.0,  0.0, -1.0,  0.0,
                                                   1.0,  0.0,  0.0,  0.0,
                                                   0.0,  0.0,  0.0,  1.0).finished();
// input cloud
common::PointCloudPtr cloud;
// outputs of loam->Run()
std::vector<oh_my_loam::TPointCloudConstPtr> cloud_vector;
Eigen::Matrix4f pose;

static oh_my_loam::TPointCloudPtr loam_output_map(new oh_my_loam::TPointCloud);


bool sb_new_slam_configuration(SLAMBenchLibraryHelper * slam_settings) {

    slam_settings->addParameter(TypedParameter<std::string>("configs", "configuration", "path to configuration YAML file", &yaml_path, &default_yaml_path));

    return true;
}


bool sb_init_slam_system(SLAMBenchLibraryHelper *slam_settings) {
    
    // Declare Outputs
    loam_pose_output = new slambench::outputs::Output("LOAM Pose", slambench::values::VT_POSE, true);

    loam_pointcloud_output = new slambench::outputs::Output("LOAM PointCloud", slambench::values::VT_POINTCLOUD, true);
    loam_pointcloud_output->SetKeepOnlyMostRecent(true);

    slam_settings->GetOutputManager().RegisterOutput(loam_pose_output);
    slam_settings->GetOutputManager().RegisterOutput(loam_pointcloud_output);

    // Inspect sensors
    lidar_sensor = (slambench::io::LidarSensor*)slam_settings->get_sensors().GetSensor(slambench::io::LidarSensor::kLidarType);
    if (lidar_sensor == nullptr) {
        std::cerr << "Invalid sensors found, Lidar not found." << std::endl;
        delete loam_pose_output;
        delete loam_pointcloud_output;
        return false;
    }

    // Start LOAM
    common::YAMLConfig::Instance()->Init(yaml_path);

    std::string lidar = common::YAMLConfig::Instance()->Get<std::string>("lidar");
    dataset_name = common::YAMLConfig::Instance()->Get<std::string>("dataset_name");
    show_point_cloud = common::YAMLConfig::Instance()->Get<bool>("show_point_cloud");
    show_full_map = common::YAMLConfig::Instance()->Get<bool>("show_full_map");
    point_cloud_ratio = common::YAMLConfig::Instance()->Get<int>("point_cloud_ratio");

    std::cout << "Use " << dataset_name << " Dataset" << std::endl;

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

        void* rawData = s->GetData();
        size_t dataSize = s->GetVariableSize(); 

        char* byteData = reinterpret_cast<char*>(rawData);

        cloud = common::PointCloudPtr(new common::PointCloud);

        for (size_t i = 0; i < dataSize; i += sizeof(pcl::PointXYZI)) {
            pcl::PointXYZI* point4D = reinterpret_cast<pcl::PointXYZI*>(byteData + i);

            pcl::PointXYZ point3D;
            point3D.x = point4D->x;
            point3D.y = point4D->y;
            point3D.z = point4D->z;
            cloud->points.push_back(point3D);
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

    if (loam_pose_output->IsActive()) {
        std::lock_guard<FastLock> lock (lib->GetOutputManager().GetLock());
		loam_pose_output->AddPoint(ts, new slambench::values::PoseValue(align_mat * pose));
    }

    if (loam_pointcloud_output->IsActive() && show_point_cloud) {

        Eigen::Matrix4f pose_map = align_mat * pose;

        oh_my_loam::TPointCloudConstPtr cloud_corn = cloud_vector.at(0);
        oh_my_loam::TPointCloudConstPtr cloud_surf = cloud_vector.at(1);

        oh_my_loam::TPointCloudPtr cloud_corn_trans(new oh_my_loam::TPointCloud);
        oh_my_loam::TPointCloudPtr cloud_surf_trans(new oh_my_loam::TPointCloud);

        pcl::transformPointCloud(*cloud_corn, *cloud_corn_trans, pose_map);
        pcl::transformPointCloud(*cloud_surf, *cloud_surf_trans, pose_map);

        auto slambench_point_cloud = new slambench::values::PointCloudValue();

        if (show_full_map) {

            loam_output_map->points.insert(loam_output_map->end(), cloud_corn_trans->begin(), cloud_corn_trans->end());
            loam_output_map->points.insert(loam_output_map->end(), cloud_surf_trans->begin(), cloud_surf_trans->end());
            loam_output_map->width = loam_output_map->points.size();  // Don't forget to update the width

            int count = 0;
            for(const auto &p : *loam_output_map) {
                if (count % point_cloud_ratio == 0) slambench_point_cloud->AddPoint(slambench::values::Point3DF(p.x, p.y, p.z));
                count++;
            }

        } else {
            
            for(const auto &p : *cloud_corn_trans) {
                slambench_point_cloud->AddPoint(slambench::values::Point3DF(p.x, p.y, p.z));
            }
            for(const auto &p : *cloud_surf_trans) {
                slambench_point_cloud->AddPoint(slambench::values::Point3DF(p.x, p.y, p.z));
            }
        }

        // Take lock only after generating the map
        std::lock_guard<FastLock> lock (lib->GetOutputManager().GetLock());
        loam_pointcloud_output->AddPoint(ts, slambench_point_cloud);
    }

    return true;
}


bool sb_clean_slam_system() {
    delete loam_pose_output;
    delete loam_pointcloud_output;
    delete lidar_sensor;
    return true;
}
