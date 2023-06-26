#include <iostream>
#include <time.h>
#include <iterator>
#include <stdio.h>
#include <string>
#include <vector>
#include <dirent.h>
#include <chrono>

#include "fastLoam.h"
#include "yawCalib.h"
#include "rpzCalib.h"
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <thread>
using namespace std::chrono_literals;

const char usage[] = {
    " ./bin/run_lidar2car <dataset_folder> <output_dir> <start_frame>(optional) <end_frame>(optional)\n"
    "PARAMS:\n"
    "  dataset_folder: path to lidar dataset directory \n"
    "  output_dir: folder to save output files.\n"
    "  start_frame, end_frame: data range. \n"
    "EXAMPLE:\n"
    "  ./bin/run_lidar2car ./data/example ./output/ \n"
    "  ./bin/run_lidar2car ./data/example ./output/ 0 1200\n"};

int test_filter_z(std::string pcd_path, double z_min, double z_max)
{
    // load pcd
    // std::string pcd_path = "/media/levin/DATA/zf/semantic_seg/temp_calib/0609/HesaiRear_pcd/00000046.pcd";
    PointCloudPtr cloud(new PointCloud());
    if (pcl::io::loadPCDFile(pcd_path, *cloud) < 0)
    {
        std::cout << "cannot open pcd_file: " << pcd_path << std::endl;
        exit(1);
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);

    PCUtil::ZRangeFilter(cloud, z_min, z_max, cloud_filtered);
        
    // Create the filtering object
    // pcl::PassThrough<pcl::PointXYZI> pass;
    // pass.setInputCloud (cloud);
    // pass.setFilterFieldName ("z");
    // pass.setFilterLimits (-2.4, -1.75);
    // //pass.setNegative (true);
    // pass.filter (*cloud_filtered);

    //get statistics
    pcl::MomentOfInertiaEstimation <pcl::PointXYZI> feature_extractor;
    feature_extractor.setInputCloud (cloud_filtered);
    feature_extractor.compute ();
    pcl::PointXYZI min_point_AABB;
    pcl::PointXYZI max_point_AABB;
    feature_extractor.getAABB (min_point_AABB, max_point_AABB);    
    std::cout<< "min_point_AABB = " <<min_point_AABB <<std::endl;
    std::cout<< "max_point_AABB = " <<max_point_AABB <<std::endl;


    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    viewer->addPointCloud<pcl::PointXYZI> (cloud_filtered, "sample cloud");
    viewer->addCube (min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");



    // pcl::visualization::CloudViewer viewer("Cloud Viewer");
    // viewer.showCloud (cloud_filtered);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(100ms);
    }
    

    return 0;
}
int main(int argc, char **argv)
{
    if (argc == 4){
        //check range z fitler result
        std::string pcd_path = argv[1];
        double z_min = std::stod(argv[2]); 
        double z_max = std::stod(argv[3]);
        return test_filter_z(pcd_path, z_min, z_max);
    }
    


    if (argc != 3 && argc != 5)
    {
        std::cerr << "Usage:" << usage;
        return 1;
    }
    std::string dataset_folder = argv[1];
    std::string output_dir = argv[2];


    int start_frame = 0;
    int end_frame = INT_MAX;
    RPCalib calibrator1(output_dir);
    if (argc == 5)
    {
        calibrator1.z_min_ = std::stod(argv[3]); 
        calibrator1.z_max_ = std::stod(argv[4]);
    }

    std::vector<Eigen::Matrix4d> lidar_pose;
    // RunFastLoam(dataset_folder, output_dir, lidar_pose, start_frame, end_frame);
    // std::cout << "Frame num:" << lidar_pose.size() << std::endl;
    
    calibrator1.LoadData(dataset_folder, lidar_pose, start_frame, end_frame);
    Eigen::Matrix4d extrinsic = Eigen::Matrix4d::Identity();
    double roll = 0, pitch = 0;
    if (calibrator1.Calibrate(extrinsic))
    {
        roll = Util::GetRoll(extrinsic);
        pitch = Util::GetPitch(extrinsic);
        std::cout << "Final result:" << std::endl
                  << "roll = " << rad2deg(roll) << " degree" << std::endl
                  << "pitch = " << rad2deg(pitch) << " degree" << std::endl
                  << "z = " << extrinsic(2, 3) << " m" << std::endl;
    }
    else
    {
        std::cout << "No valid data for calibrating pitch and roll." << std::endl;
    }

    // YawCalib calibrator2(output_dir);
    // calibrator2.LoadData(lidar_pose);

    // double yaw = 0;
    // if (calibrator2.Calibrate())
    // {
    //     yaw = calibrator2.GetFinalYaw();
    //     std::cout << "Final result:" << std::endl
    //               << "yaw = " << rad2deg(yaw) << " degree" << std::endl;
    // }
    // else{
    //     std::cout << "No valid data for calibrating yaw." << std::endl;
    // }

    // Eigen::Matrix3d rotation;
    // rotation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    //         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    //         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    // extrinsic.block<3, 3>(0, 0) = rotation;
    // SaveExtrinsic(extrinsic, output_dir);
}
