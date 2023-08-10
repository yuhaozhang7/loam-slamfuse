/*

 Copyright (c) 2023 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

*/

#include <Parameters.h>
#include <SLAMBenchAPI.h>
#include <io/SLAMFrame.h>
#include <io/sensor/LidarSensor.h>
#include <io/sensor/CameraSensor.h>
#include <io/sensor/CameraSensorFinder.h>
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

// Default Parameters
const std::string default_lidar_name = "VLP";
// TODO: change the path to fit Docker
const std::string default_yaml_path = "/home/yuhao/loam/configs/default.yaml";
const std::string dirname = "/mnt/d/Download/Dataset/Kitti/2011_09_30/2011_09_30_drive_0027_sync/velodyne_points/pcd/";

// Parameters
std::string lidar_name;
std::string yaml_path;

// Sensors
slambench::io::LidarSensor *lidar_sensor;
slambench::io::CameraSensor *grey_sensor;

size_t frame_id = 0;
static sb_uint2 inputSize;
std::vector<unsigned char> grey_image;

slambench::TimeStamp last_frame_timestamp;
double current_timestamp;

// Outputs
slambench::outputs::Output *pose_output;
slambench::outputs::Output *pointcloud_output;
slambench::outputs::Output *grey_frame_output;

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

    slam_settings->addParameter(TypedParameter<std::string>("", "lidar-name", "name of lidar sensor", &lidar_name, &default_lidar_name));
    slam_settings->addParameter(TypedParameter<std::string>("", "configuration", "path to configuration YAML file", &yaml_path, &default_yaml_path));

    return true;
}


bool sb_init_slam_system(SLAMBenchLibraryHelper *slam_settings) {
    
    // Declare Outputs
    pose_output = new slambench::outputs::Output("Pose", slambench::values::VT_POSE, true);

    pointcloud_output = new slambench::outputs::Output("PointCloud", slambench::values::VT_POINTCLOUD, true);
    pointcloud_output->SetKeepOnlyMostRecent(true);

    grey_frame_output = new slambench::outputs::Output("Grey Frame", slambench::values::VT_FRAME);
    grey_frame_output->SetKeepOnlyMostRecent(true);

    slam_settings->GetOutputManager().RegisterOutput(pose_output);
    slam_settings->GetOutputManager().RegisterOutput(pointcloud_output);
    slam_settings->GetOutputManager().RegisterOutput(grey_frame_output);

    // Inspect sensors
    lidar_sensor = (slambench::io::LidarSensor*)slam_settings->get_sensors().GetSensor(slambench::io::LidarSensor::kLidarType);
    if (lidar_sensor == nullptr) {
        std::cerr << "Invalid sensors found, Lidar not found." << std::endl;
        return false;
    }

    slambench::io::CameraSensorFinder sensor_finder;
	grey_sensor = sensor_finder.FindOne(slam_settings->get_sensors(), {{"camera_type", "grey"}});
    if (grey_sensor == nullptr) {
		std::cerr << "Invalid sensors found, Grey not found." << std::endl;
		delete pose_output;
        delete pointcloud_output;
        delete grey_frame_output;
		return false;
	}
	
	// check sensor frame and pixel format
	if(grey_sensor->PixelFormat != slambench::io::pixelformat::G_I_8) {
		std::cerr << "Grey sensor is not in G_I_8 format" << std::endl;
        delete pose_output;
        delete pointcloud_output;
        delete grey_frame_output;
		return false;
	}

    inputSize = make_sb_uint2(grey_sensor->Width, grey_sensor->Height);
    grey_image.resize(grey_sensor->Width * grey_sensor->Height);

    // Start LOAM
    common::YAMLConfig::Instance()->Init(default_yaml_path);

    bool is_log_to_file = common::YAMLConfig::Instance()->Get<bool>("log_to_file");
    std::string log_path = common::YAMLConfig::Instance()->Get<std::string>("log_path");
    std::string lidar = common::YAMLConfig::Instance()->Get<std::string>("lidar");

    if (!loam.Init()) {
        std::cerr << "Failed to initilize slam system." << std::endl;
    }

    std::cout << "LOAM start..., lidar = " << default_lidar_name << std::endl;

    return true;
}


bool sb_update_frame(SLAMBenchLibraryHelper *slam_settings , slambench::io::SLAMFrame *s) {

	if (s->FrameSensor == lidar_sensor) {

        last_frame_timestamp = s->Timestamp;
        current_timestamp = static_cast<double>(s->Timestamp.S) + static_cast<double>(s->Timestamp.Ns) / 1e9;

        char* data = (char*)s->GetData();
        uint32_t count = *(uint32_t*)data;
        float *fdata = (float*)(data+4);

        cloud = common::PointCloudPtr(new common::PointCloud);

        for(uint32_t i = 0; i < count; ++i) {
            float x = fdata[i*3];
            float y = fdata[i*3+1];
            float z = fdata[i*3+2];
            common::Point point;
            point.x = x;
            point.y = y;
            point.z = z;
            cloud->points.push_back(point);
        }
        cloud->width = cloud->points.size();
        cloud->height = 1;
        
        /*
        cloud = common::PointCloudPtr(new common::PointCloud);
        std::stringstream tmp_filename;
        tmp_filename << std::setw(10) << std::setfill('0') << frame_id;
        std::string lidar_file_pcd = tmp_filename.str() + ".pcd";
        lidar_file_pcd = dirname + lidar_file_pcd;
        std::cout << lidar_file_pcd << std::endl;

        pcl::io::loadPCDFile(lidar_file_pcd, *cloud);
        */
        
        return true;
	}

    if(s->FrameSensor == grey_sensor) {
        std::cout << "start memcpy" << std::endl;
		memcpy(grey_image.data(), s->GetData(), s->GetSize());
        std::cout << "memcpy is good" << std::endl;
		s->FreeData();
		last_frame_timestamp = s->Timestamp;
		return true;
	}
	
	return false;
}


bool sb_process_once(SLAMBenchLibraryHelper *slam_settings) {

    std::cout << "\nOhmyloam: frame_id = " << ++frame_id
              << ", timestamp = " << FMT_TIMESTAMP(current_timestamp)
              << ", point_number = " << cloud->size() << std::endl;
    
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

    if(grey_frame_output->IsActive()) {
		std::lock_guard<FastLock> lock (lib->GetOutputManager().GetLock());
		grey_frame_output->AddPoint(last_frame_timestamp, new slambench::values::FrameValue(inputSize.x, inputSize.y, slambench::io::pixelformat::G_I_8, (void*) &(grey_image.at(0))));
	}

    return true;
}


bool sb_clean_slam_system() {
    delete pose_output;
    delete pointcloud_output;
    delete grey_frame_output;
    delete lidar_sensor;
    delete grey_sensor;
    return true;
}
