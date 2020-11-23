#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

//#include <opencv/cv.h>
#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

using namespace std;

typedef pcl::PointXYZI PointType;

class ParamServer: public rclcpp::Node
{
public:

    // rclcpp::NodeHandle nh; // todo

    std::string robot_id;

    //Topics
    string pointCloudTopic;
    string imuTopic;
    string odomTopic;
    string gpsTopic;

    //Frames
    string lidarFrame;
    string baselinkFrame;
    string odometryFrame;
    string mapFrame;

    // GPS Settings
    bool useImuHeadingInitialization;
    bool useGpsElevation;
    float gpsCovThreshold;
    float poseCovThreshold;

    // Save pcd
    bool savePCD;
    string savePCDDirectory;

    // Velodyne Sensor Configuration: Velodyne
    int N_SCAN;
    int Horizon_SCAN;
    string timeField;
    int downsampleRate;
    float lidarMinRange;
    float lidarMaxRange;

    // IMU
    float imuAccNoise;
    float imuGyrNoise;
    float imuAccBiasN;
    float imuGyrBiasN;
    float imuGravity;
    float imuRPYWeight;
    vector<double> extRotV;
    vector<double> extRPYV;
    vector<double> extTransV;
    Eigen::Matrix3d extRot;
    Eigen::Matrix3d extRPY;
    Eigen::Vector3d extTrans;
    Eigen::Quaterniond extQRPY;

    // LOAM
    float edgeThreshold;
    float surfThreshold;
    int edgeFeatureMinValidNum;
    int surfFeatureMinValidNum;

    // voxel filter paprams
    float odometrySurfLeafSize;
    float mappingCornerLeafSize;
    float mappingSurfLeafSize ;

    float z_tollerance; 
    float rotation_tollerance;

    // CPU Params
    int numberOfCores;
    double mappingProcessInterval;

    // Surrounding map
    float surroundingkeyframeAddingDistThreshold; 
    float surroundingkeyframeAddingAngleThreshold; 
    float surroundingKeyframeDensity;
    float surroundingKeyframeSearchRadius;
    
    // Loop closure
    bool  loopClosureEnableFlag;
    float loopClosureFrequency;
    int   surroundingKeyframeSize;
    float historyKeyframeSearchRadius;
    float historyKeyframeSearchTimeDiff;
    int   historyKeyframeSearchNum;
    float historyKeyframeFitnessScore;

    // global map visualization radius
    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;

    ParamServer(std::string node_name, const rclcpp::NodeOptions & options):Node(node_name, options)
    {
        //nh.param<std::string>("/robot_id", robot_id, "roboat");
        declare_parameter("/robot_id", "roboat");
        get_parameter("/robot_id", robot_id);
        //nh.param<std::string>("lio_sam/pointCloudTopic", pointCloudTopic, "points_raw");
        declare_parameter("lio_sam/pointCloudTopic", "points_raw");
        get_parameter("lio_sam/pointCloudTopic", pointCloudTopic);
        //nh.param<std::string>("lio_sam/imuTopic", imuTopic, "imu_correct");
        declare_parameter("lio_sam/imuTopic", "imu_correct");
        get_parameter("lio_sam/imuTopic", imuTopic);
        //nh.param<std::string>("lio_sam/odomTopic", odomTopic, "odometry/imu");
        declare_parameter("lio_sam/odomTopic", "odometry/imu");
        get_parameter("lio_sam/odomTopic", odomTopic);
        //nh.param<std::string>("lio_sam/gpsTopic", gpsTopic, "odometry/gps");
        declare_parameter("lio_sam/gpsTopic", "odometry/gps");
        get_parameter("lio_sam/gpsTopic", gpsTopic);

        //nh.param<std::string>("lio_sam/lidarFrame", lidarFrame, "base_link");
        declare_parameter("lio_sam/lidarFrame", "base_link");
        get_parameter("lio_sam/lidarFrame", lidarFrame);
        //nh.param<std::string>("lio_sam/baselinkFrame", baselinkFrame, "base_link");
        declare_parameter("lio_sam/baselinkFrame", "base_link");
        get_parameter("lio_sam/baselinkFrame", baselinkFrame);
        //nh.param<std::string>("lio_sam/odometryFrame", odometryFrame, "odom");
        declare_parameter("lio_sam/odometryFrame", "odom");
        get_parameter("lio_sam/odometryFrame", odometryFrame);
        //nh.param<std::string>("lio_sam/mapFrame", mapFrame, "map");
        declare_parameter("lio_sam/mapFrame", "map");
        get_parameter("lio_sam/mapFrame", mapFrame);

        //nh.param<bool>("lio_sam/useImuHeadingInitialization", useImuHeadingInitialization, false);
        declare_parameter("lio_sam/useImuHeadingInitialization", false);
        get_parameter("lio_sam/useImuHeadingInitialization", useImuHeadingInitialization);
        //nh.param<bool>("lio_sam/useGpsElevation", useGpsElevation, false);
        declare_parameter("lio_sam/useGpsElevation", false);
        get_parameter("lio_sam/useGpsElevation", useGpsElevation);
        //nh.param<float>("lio_sam/gpsCovThreshold", gpsCovThreshold, 2.0);
        declare_parameter("lio_sam/gpsCovThreshold", 2.0);
        get_parameter("lio_sam/gpsCovThreshold", gpsCovThreshold);
        //nh.param<float>("lio_sam/poseCovThreshold", poseCovThreshold, 25.0);
        declare_parameter("lio_sam/poseCovThreshold", 25.0);
        get_parameter("lio_sam/poseCovThreshold", poseCovThreshold);

        //nh.param<bool>("lio_sam/savePCD", savePCD, false);
        declare_parameter("lio_sam/savePCD", false);
        get_parameter("lio_sam/savePCD", savePCD);
        //nh.param<std::string>("lio_sam/savePCDDirectory", savePCDDirectory, "/Downloads/LOAM/");
        declare_parameter("lio_sam/savePCDDirectory", "/Downloads/LOAM/");
        get_parameter("lio_sam/savePCDDirectory", savePCDDirectory);

        //nh.param<int>("lio_sam/N_SCAN", N_SCAN, 16);
        declare_parameter("lio_sam/N_SCAN", 16);
        get_parameter("lio_sam/N_SCAN", N_SCAN);
        //nh.param<int>("lio_sam/Horizon_SCAN", Horizon_SCAN, 1800);
        declare_parameter("lio_sam/Horizon_SCAN", 1800);
        get_parameter("lio_sam/Horizon_SCAN", Horizon_SCAN);
        //nh.param<std::string>("lio_sam/timeField", timeField, "time");
        declare_parameter("lio_sam/timeField", "time");
        get_parameter("lio_sam/timeField", timeField);
        //nh.param<int>("lio_sam/downsampleRate", downsampleRate, 1);
        declare_parameter("lio_sam/downsampleRate", 1);
        get_parameter("lio_sam/downsampleRate", downsampleRate);
        //nh.param<float>("lio_sam/lidarMinRange", lidarMinRange, 1.0);
        declare_parameter("lio_sam/lidarMinRange", 1.0);
        get_parameter("lio_sam/lidarMinRange", lidarMinRange);
        //nh.param<float>("lio_sam/lidarMaxRange", lidarMaxRange, 1000.0);
        declare_parameter("lio_sam/lidarMaxRange", 1000.0);
        get_parameter("lio_sam/lidarMaxRange", lidarMaxRange);

        //nh.param<float>("lio_sam/imuAccNoise", imuAccNoise, 0.01);
        declare_parameter("lio_sam/imuAccNoise", 0.01);
        get_parameter("lio_sam/imuAccNoise", imuAccNoise);
        //nh.param<float>("lio_sam/imuGyrNoise", imuGyrNoise, 0.001);
        declare_parameter("lio_sam/imuGyrNoise", 001);
        get_parameter("lio_sam/imuGyrNoise", imuGyrNoise);
        //nh.param<float>("lio_sam/imuAccBiasN", imuAccBiasN, 0.0002);
        declare_parameter("lio_sam/imuAccBiasN", 0.0002);
        get_parameter("lio_sam/imuAccBiasN", imuAccBiasN);
        //nh.param<float>("lio_sam/imuGyrBiasN", imuGyrBiasN, 0.00003);
        declare_parameter("lio_sam/imuGyrBiasN", 0.00003);
        get_parameter("lio_sam/imuGyrBiasN", imuGyrBiasN);     
        //nh.param<float>("lio_sam/imuGravity", imuGravity, 9.80511);
        declare_parameter("lio_sam/imuGravity", 9.80511);
        get_parameter("lio_sam/imuGravity", imuGravity);
        //nh.param<float>("lio_sam/imuRPYWeight", imuRPYWeight, 0.01);
        declare_parameter("lio_sam/imuRPYWeight", 0.01);
        get_parameter("lio_sam/imuRPYWeight", imuRPYWeight);
        //nh.param<vector<double>>("lio_sam/extrinsicRot", extRotV, vector<double>());
        declare_parameter("lio_sam/extrinsicRot", vector<double>());
        get_parameter("lio_sam/extrinsicRot", extRotV);
        //nh.param<vector<double>>("lio_sam/extrinsicRPY", extRPYV, vector<double>());
        declare_parameter("lio_sam/extrinsicRPY", vector<double>());
        get_parameter("lio_sam/extrinsicRPY", extRPYV);
        //nh.param<vector<double>>("lio_sam/extrinsicTrans", extTransV, vector<double>());
        declare_parameter("lio_sam/extrinsicTrans", vector<double>());
        get_parameter("lio_sam/extrinsicTrans", extTransV);
        
        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
        extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
        extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
        extQRPY = Eigen::Quaterniond(extRPY);

        //nh.param<float>("lio_sam/edgeThreshold", edgeThreshold, 0.1);
        declare_parameter("lio_sam/edgeThreshold", 0.1);
        get_parameter("lio_sam/edgeThreshold", edgeThreshold);
        //nh.param<float>("lio_sam/surfThreshold", surfThreshold, 0.1);
        declare_parameter("lio_sam/surfThreshold", 0.1);
        get_parameter("lio_sam/surfThreshold", surfThreshold);
        //nh.param<int>("lio_sam/edgeFeatureMinValidNum", edgeFeatureMinValidNum, 10);
        declare_parameter("lio_sam/edgeFeatureMinValidNum", 10);
        get_parameter("lio_sam/edgeFeatureMinValidNum", edgeFeatureMinValidNum);
        //nh.param<int>("lio_sam/surfFeatureMinValidNum", surfFeatureMinValidNum, 100);
        declare_parameter("lio_sam/surfFeatureMinValidNum", 100);
        get_parameter("lio_sam/surfFeatureMinValidNum", surfFeatureMinValidNum);

        //nh.param<float>("lio_sam/odometrySurfLeafSize", odometrySurfLeafSize, 0.2);
        declare_parameter("lio_sam/odometrySurfLeafSize", 0.2);
        get_parameter("lio_sam/odometrySurfLeafSize", odometrySurfLeafSize);
        //nh.param<float>("lio_sam/mappingCornerLeafSize", mappingCornerLeafSize, 0.2);
        declare_parameter("lio_sam/mappingCornerLeafSize", 0.2);
        get_parameter("lio_sam/mappingCornerLeafSize", mappingCornerLeafSize);
        //nh.param<float>("lio_sam/mappingSurfLeafSize", mappingSurfLeafSize, 0.2);
        declare_parameter("lio_sam/mappingSurfLeafSize", 0.2);
        get_parameter("lio_sam/mappingSurfLeafSize", mappingSurfLeafSize);

        //nh.param<float>("lio_sam/z_tollerance", z_tollerance, FLT_MAX);
        declare_parameter("lio_sam/z_tollerance", FLT_MAX);
        get_parameter("lio_sam/z_tollerance", z_tollerance);
        //nh.param<float>("lio_sam/rotation_tollerance", rotation_tollerance, FLT_MAX);
        declare_parameter("lio_sam/rotation_tollerance", FLT_MAX);
        get_parameter("lio_sam/rotation_tollerance", rotation_tollerance);
        
        //nh.param<int>("lio_sam/numberOfCores", numberOfCores, 2);
        declare_parameter("lio_sam/numberOfCores", 2);
        get_parameter("lio_sam/numberOfCores", numberOfCores);
        //nh.param<double>("lio_sam/mappingProcessInterval", mappingProcessInterval, 0.15);
        declare_parameter("lio_sam/mappingProcessInterval", 0.15);
        get_parameter("lio_sam/mappingProcessInterval", mappingProcessInterval);

        //nh.param<float>("lio_sam/surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 1.0);
        declare_parameter("lio_sam/surroundingkeyframeAddingDistThreshold", 1.0);
        get_parameter("lio_sam/surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold);
        //nh.param<float>("lio_sam/surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2);
        declare_parameter("lio_sam/surroundingkeyframeAddingAngleThreshold", 0.2);
        get_parameter("lio_sam/surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold);
        //nh.param<float>("lio_sam/surroundingKeyframeDensity", surroundingKeyframeDensity, 1.0);
        declare_parameter("lio_sam/surroundingKeyframeDensity", 1.0);
        get_parameter("lio_sam/surroundingKeyframeDensity", surroundingKeyframeDensity);
        //nh.param<float>("lio_sam/surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0);
        declare_parameter("lio_sam/surroundingKeyframeSearchRadius", 50.0);
        get_parameter("lio_sam/surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius);

        //nh.param<bool>("lio_sam/loopClosureEnableFlag", loopClosureEnableFlag, false);
        declare_parameter("lio_sam/loopClosureEnableFlag", false);
        get_parameter("lio_sam/loopClosureEnableFlag", loopClosureEnableFlag);
        //nh.param<float>("lio_sam/loopClosureFrequency", loopClosureFrequency, 1.0);
        declare_parameter("lio_sam/loopClosureFrequency", 1.0);
        get_parameter("lio_sam/loopClosureFrequency", loopClosureFrequency);
        //nh.param<int>("lio_sam/surroundingKeyframeSize", surroundingKeyframeSize, 50);
        declare_parameter("lio_sam/surroundingKeyframeSize", 50);
        get_parameter("lio_sam/surroundingKeyframeSize", surroundingKeyframeSize);
        //nh.param<float>("lio_sam/historyKeyframeSearchRadius", historyKeyframeSearchRadius, 10.0);
        declare_parameter("lio_sam/historyKeyframeSearchRadius", 10.0);
        get_parameter("lio_sam/historyKeyframeSearchRadius", historyKeyframeSearchRadius);
        //nh.param<float>("lio_sam/historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0);
        declare_parameter("lio_sam/historyKeyframeSearchTimeDiff", 30.0);
        get_parameter("lio_sam/historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff);
        //nh.param<int>("lio_sam/historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
        declare_parameter("lio_sam/historyKeyframeSearchNum", 25);
        get_parameter("lio_sam/historyKeyframeSearchNum", historyKeyframeSearchNum);
        //nh.param<float>("lio_sam/historyKeyframeFitnessScore", historyKeyframeFitnessScore, 0.3);
        declare_parameter("lio_sam/historyKeyframeFitnessScore", 0.3);
        get_parameter("lio_sam/historyKeyframeFitnessScore", historyKeyframeFitnessScore);

        //nh.param<float>("lio_sam/globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1e3);
        declare_parameter("lio_sam/globalMapVisualizationSearchRadius", 1e3);
        get_parameter("lio_sam/globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius);
        //nh.param<float>("lio_sam/globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.0);
        declare_parameter("lio_sam/globalMapVisualizationPoseDensity", 10.0);
        get_parameter("lio_sam/globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity);
        //nh.param<float>("lio_sam/globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.0);
        declare_parameter("lio_sam/globalMapVisualizationLeafSize", 1.0);
        get_parameter("lio_sam/globalMapVisualizationLeafSize", globalMapVisualizationLeafSize);

        usleep(100);
    }

    sensor_msgs::msg::Imu imuConverter(const sensor_msgs::msg::Imu& imu_in)
    {
        sensor_msgs::msg::Imu imu_out = imu_in;
        // rotate acceleration
        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        acc = extRot * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();
        // rotate gyroscope
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = extRot * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();
        // rotate roll pitch yaw
        Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
        Eigen::Quaterniond q_final = q_from * extQRPY;
        imu_out.orientation.x = q_final.x();
        imu_out.orientation.y = q_final.y();
        imu_out.orientation.z = q_final.z();
        imu_out.orientation.w = q_final.w();

        if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
        {
            RCLCPP_ERROR(get_logger(), "Invalid quaternion, please use a 9-axis IMU!");
            rclcpp::shutdown();
        }

        return imu_out;
    }
};


//sensor_msgs::msg::PointCloud2 publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, rclcpp::Time thisStamp, std::string thisFrame)
//{
//    sensor_msgs::msg::PointCloud2 tempCloud;
//    pcl::toROSMsg(*thisCloud, tempCloud);
//    tempCloud.header.stamp = thisStamp;
//    tempCloud.header.frame_id = thisFrame;
//    if (thisPub->get_subscription_count() != 0)
//        thisPub->publish(tempCloud);
//    return tempCloud;
//}

template<typename T>
double ROS_TIME(T msg)
{
    return msg->header.stamp.sec;
}


template<typename T>
void imuAngular2rosAngular(sensor_msgs::msg::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
{
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}


template<typename T>
void imuAccel2rosAccel(sensor_msgs::msg::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z)
{
    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}


template<typename T>
void imuRPY2rosRPY(sensor_msgs::msg::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw)
{
    double imuRoll, imuPitch, imuYaw;
    tf2::Quaternion orientation;
    //tf2::quaternionMsgToTF(thisImuMsg->orientation, orientation);
    tf2::fromMsg(thisImuMsg->orientation, orientation);
    tf2::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

    *rosRoll = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw = imuYaw;
}


float pointDistance(PointType p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}


float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

#endif
