#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

#include <iostream>
using std::cout; 	using std::endl;
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
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

    ParamServer(std::string node_name, const rclcpp::NodeOptions & options) : Node(node_name, options)
    {
        declare_parameter("/robot_id", "roboat");
        get_parameter("/robot_id", robot_id);
        declare_parameter("lio_sam_ros2/pointCloudTopic", "points_raw");
        get_parameter("lio_sam_ros2/pointCloudTopic", pointCloudTopic);
        declare_parameter("lio_sam_ros2/imuTopic", "imu_correct");
        get_parameter("lio_sam_ros2/imuTopic", imuTopic);
        declare_parameter("lio_sam_ros2/odomTopic", "odometry/imu");
        get_parameter("lio_sam_ros2/odomTopic", odomTopic);
        declare_parameter("lio_sam_ros2/gpsTopic", "odometry/gps");
        get_parameter("lio_sam_ros2/gpsTopic", gpsTopic);

        declare_parameter("lio_sam_ros2/lidarFrame", "base_link");
        get_parameter("lio_sam_ros2/lidarFrame", lidarFrame);
        declare_parameter("lio_sam_ros2/baselinkFrame", "base_link");
        get_parameter("lio_sam_ros2/baselinkFrame", baselinkFrame);
        declare_parameter("lio_sam_ros2/odometryFrame", "odom");
        get_parameter("lio_sam_ros2/odometryFrame", odometryFrame);
        declare_parameter("lio_sam_ros2/mapFrame", "map");
        get_parameter("lio_sam_ros2/mapFrame", mapFrame);

        declare_parameter("lio_sam_ros2/useImuHeadingInitialization", false);
        get_parameter("lio_sam_ros2/useImuHeadingInitialization", useImuHeadingInitialization);
        declare_parameter("lio_sam_ros2/useGpsElevation", false);
        get_parameter("lio_sam_ros2/useGpsElevation", useGpsElevation);
        declare_parameter("lio_sam_ros2/gpsCovThreshold", 2.0);
        get_parameter("lio_sam_ros2/gpsCovThreshold", gpsCovThreshold);
        declare_parameter("lio_sam_ros2/poseCovThreshold", 25.0);
        get_parameter("lio_sam_ros2/poseCovThreshold", poseCovThreshold);
        
        declare_parameter("lio_sam_ros2/savePCD", false);
        get_parameter("lio_sam_ros2/savePCD", savePCD);
        
        declare_parameter("lio_sam_ros2/savePCDDirectory", "/Downloads/LOAM/");
        get_parameter("lio_sam_ros2/savePCDDirectory", savePCDDirectory);

        declare_parameter("lio_sam_ros2/N_SCAN", 16);
        get_parameter("lio_sam_ros2/N_SCAN", N_SCAN);
        declare_parameter("lio_sam_ros2/Horizon_SCAN", 1800);
        get_parameter("lio_sam_ros2/Horizon_SCAN", Horizon_SCAN);
        declare_parameter("lio_sam_ros2/timeField", "time");
        get_parameter("lio_sam_ros2/timeField", timeField);
        declare_parameter("lio_sam_ros2/downsampleRate", 1);
        get_parameter("lio_sam_ros2/downsampleRate", downsampleRate);
        declare_parameter("lio_sam_ros2/lidarMinRange", 1.0);
        get_parameter("lio_sam_ros2/lidarMinRange", lidarMinRange);
        declare_parameter("lio_sam_ros2/lidarMaxRange", 1000.0);
        get_parameter("lio_sam_ros2/lidarMaxRange", lidarMaxRange);

        declare_parameter("lio_sam_ros2/imuAccNoise", 0.01);
        get_parameter("lio_sam_ros2/imuAccNoise", imuAccNoise);
        declare_parameter("lio_sam_ros2/imuGyrNoise", 0.01);
        get_parameter("lio_sam_ros2/imuGyrNoise", imuGyrNoise);
        declare_parameter("lio_sam_ros2/imuAccBiasN", 0.0002);
        get_parameter("lio_sam_ros2/imuAccBiasN", imuAccBiasN);
        declare_parameter("lio_sam_ros2/imuGyrBiasN", 0.00003);
        get_parameter("lio_sam_ros2/imuGyrBiasN", imuGyrBiasN);
        declare_parameter("lio_sam_ros2/imuGravity", 9.80511);
        get_parameter("lio_sam_ros2/imuGravity", imuGravity);
        declare_parameter("lio_sam_ros2/imuRPYWeight", 0.01);
        get_parameter("lio_sam_ros2/imuRPYWeight", imuRPYWeight);
        //declare_parameter("lio_sam_ros2/extrinsicRot", vector<double>());
        //get_parameter("lio_sam_ros2/extrinsicRot", extRotV);
        //declare_parameter("lio_sam_ros2/extrinsicRPY", vector<double>());
        //get_parameter("lio_sam_ros2/extrinsicRPY", extRPYV);
        //declare_parameter("lio_sam_ros2/extrinsicTrans", vector<double>());
        //get_parameter("lio_sam_ros2/extrinsicTrans", extTransV);

        double org_data1[] = {-1, 0, 0,
                               0, 1, 0,
                               0, 0, -1};
        std::vector < double > data1(org_data1, std::end(org_data1));
        declare_parameter("extrinsicRot", data1);
        get_parameter("extrinsicRot", extRotV);

        double org_data2[] = {0, 1, 0,
                             -1, 0, 0,
                              0, 0, 1}; 
       	std::vector < double > data2(org_data2, std::end(org_data2));
        declare_parameter("extrinsicRPY", data2);
        get_parameter("extrinsicRPY", extRPYV);
        
	double org_data3[] = {0, 0, 0};
        std::vector < double > data3(org_data3, std::end(org_data3));
        declare_parameter("extrinsicTrans", data3);
        get_parameter("extrinsicTrans", extTransV);

	//cout << " eigen begins" << endl;
	//cout << extRotV.data() << endl;
        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
	cout << " eigen ends" << endl;
        extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
        extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
        extQRPY = Eigen::Quaterniond(extRPY);
	return;

        declare_parameter("lio_sam_ros2/edgeThreshold", 0.1);
        get_parameter("lio_sam_ros2/edgeThreshold", edgeThreshold);
        declare_parameter("lio_sam_ros2/surfThreshold", 0.1);
        get_parameter("lio_sam_ros2/surfThreshold", surfThreshold);
        declare_parameter("lio_sam_ros2/edgeFeatureMinValidNum", 10);
        get_parameter("lio_sam_ros2/edgeFeatureMinValidNum", edgeFeatureMinValidNum);
        declare_parameter("lio_sam_ros2/surfFeatureMinValidNum", 100);
        get_parameter("lio_sam_ros2/surfFeatureMinValidNum", surfFeatureMinValidNum);

        declare_parameter("lio_sam_ros2/odometrySurfLeafSize", 0.2);
        get_parameter("lio_sam_ros2/odometrySurfLeafSize", odometrySurfLeafSize);
        declare_parameter("lio_sam_ros2/mappingCornerLeafSize", 0.2);
        get_parameter("lio_sam_ros2/mappingCornerLeafSize", mappingCornerLeafSize);
        declare_parameter("lio_sam_ros2/mappingSurfLeafSize", 0.2);
        get_parameter("lio_sam_ros2/mappingSurfLeafSize", mappingSurfLeafSize);

        declare_parameter("lio_sam_ros2/z_tollerance", FLT_MAX);
        get_parameter("lio_sam_ros2/z_tollerance", z_tollerance);
        declare_parameter("lio_sam_ros2/rotation_tollerance", FLT_MAX);
        get_parameter("lio_sam_ros2/rotation_tollerance", rotation_tollerance);
        
        declare_parameter("lio_sam_ros2/numberOfCores", 2);
        get_parameter("lio_sam_ros2/numberOfCores", numberOfCores);
        declare_parameter("lio_sam_ros2/mappingProcessInterval", 0.15);
        get_parameter("lio_sam_ros2/mappingProcessInterval", mappingProcessInterval);

        declare_parameter("lio_sam_ros2/surroundingkeyframeAddingDistThreshold", 1.0);
        get_parameter("lio_sam_ros2/surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold);
        declare_parameter("lio_sam_ros2/surroundingkeyframeAddingAngleThreshold", 0.2);
        get_parameter("lio_sam_ros2/surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold);
        declare_parameter("lio_sam_ros2/surroundingKeyframeDensity", 1.0);
        get_parameter("lio_sam_ros2/surroundingKeyframeDensity", surroundingKeyframeDensity);
        declare_parameter("lio_sam_ros2/surroundingKeyframeSearchRadius", 50.0);
        get_parameter("lio_sam_ros2/surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius);

        declare_parameter("lio_sam_ros2/loopClosureEnableFlag", false);
        get_parameter("lio_sam_ros2/loopClosureEnableFlag", loopClosureEnableFlag);
        declare_parameter("lio_sam_ros2/loopClosureFrequency", 1.0);
        get_parameter("lio_sam_ros2/loopClosureFrequency", loopClosureFrequency);
        declare_parameter("lio_sam_ros2/surroundingKeyframeSize", 50);
        get_parameter("lio_sam_ros2/surroundingKeyframeSize", surroundingKeyframeSize);
        declare_parameter("lio_sam_ros2/historyKeyframeSearchRadius", 10.0);
        get_parameter("lio_sam_ros2/historyKeyframeSearchRadius", historyKeyframeSearchRadius);
        declare_parameter("lio_sam_ros2/historyKeyframeSearchTimeDiff", 30.0);
        get_parameter("lio_sam_ros2/historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff);
        declare_parameter("lio_sam_ros2/historyKeyframeSearchNum", 25);
        get_parameter("lio_sam_ros2/historyKeyframeSearchNum", historyKeyframeSearchNum);
        declare_parameter("lio_sam_ros2/historyKeyframeFitnessScore", 0.3);
        get_parameter("lio_sam_ros2/historyKeyframeFitnessScore", historyKeyframeFitnessScore);

        declare_parameter("lio_sam_ros2/globalMapVisualizationSearchRadius", 1e3);
        get_parameter("lio_sam_ros2/globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius);
        declare_parameter("lio_sam_ros2/globalMapVisualizationPoseDensity", 10.0);
        get_parameter("lio_sam_ros2/globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity);
        declare_parameter("lio_sam_ros2/globalMapVisualizationLeafSize", 1.0);
        get_parameter("lio_sam_ros2/globalMapVisualizationLeafSize", globalMapVisualizationLeafSize);

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


sensor_msgs::msg::PointCloud2 publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, rclcpp::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::msg::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->get_subscription_count() != 0)
        thisPub->publish(tempCloud);
    return tempCloud;
}

template<typename T>
double ROS_TIME(T msg)
{
    return msg->header.stamp.sec + msg->header.stamp.nanosec;
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
