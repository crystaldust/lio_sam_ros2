#include "utility.hpp"
#include "lio_sam_ros2/msg/cloud_info.hpp"

// Velodyne
struct PointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,  
                                  (float, x, x) 
                                  (float, y, y) 
                                  (float, z, z) 
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring) 
                                  (float, time, time)
)

const int queueLength = 2000;

class ImageProjection : public ParamServer
{
private:
    lio_sam_ros2::msg::CloudInfo cloudInfo;

    std::mutex imuLock;
    std::mutex odoLock;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloud;
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubExtractedCloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudInfo;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
    std::deque<sensor_msgs::msg::Imu> imuQueue;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom;
    std::deque<nav_msgs::msg::Odometry> odomQueue;

    std::deque<sensor_msgs::msg::PointCloud2> cloudQueue;
    sensor_msgs::msg::PointCloud2 currentCloudMsg;
    
    double *imuTime = new double[queueLength];
    double *imuRotX = new double[queueLength];
    double *imuRotY = new double[queueLength];
    double *imuRotZ = new double[queueLength];

    int imuPointerCur;
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    pcl::PointCloud<PointType>::Ptr   fullCloud;
    pcl::PointCloud<PointType>::Ptr   extractedCloud;

    int deskewFlag;
    cv::Mat rangeMat;

    bool odomDeskewFlag;
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;
    
    double timeScanCur;
    double timeScanNext;
    std_msgs::msg::Header cloudHeader;


public:
    ImageProjection(const rclcpp::NodeOptions & options):ParamServer("lio_sam_ros2_imageProjection", options), deskewFlag(0)
    {
        auto imu_callback = [this](const sensor_msgs::msg::Imu::ConstPtr msg) -> void
        {
            imuHandler(msg);
        };
        subImu = create_subscription<sensor_msgs::msg::Imu>(imuTopic, rclcpp::SensorDataQoS(), imu_callback);
        
        auto odom_callback = [this](const nav_msgs::msg::Odometry::ConstPtr msg) -> void
        {
            odometryHandler(msg);
        };
        subOdom = create_subscription<nav_msgs::msg::Odometry>(odomTopic, rclcpp::SensorDataQoS(), odom_callback);
        
        auto lc_callback = [this](const sensor_msgs::msg::PointCloud2::ConstPtr msg) -> void
        {
            cloudHandler(msg);
        };
        subLaserCloud = create_subscription<sensor_msgs::msg::PointCloud2>(pointCloudTopic, 5, lc_callback);
        
        pubExtractedCloud = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_deskewed", 2000);
        
        pubLaserCloudInfo = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_info", 2000);
        
        allocateMemory();
        
        resetParameters();
        
        //pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }

    void allocateMemory()
    {   
        
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        
        fullCloud.reset(new pcl::PointCloud<PointType>());
        // 
        extractedCloud.reset(new pcl::PointCloud<PointType>());
        
        fullCloud->points.resize(N_SCAN*Horizon_SCAN);
        
        cloudInfo.start_ring_index.assign(N_SCAN, 0);
        cloudInfo.end_ring_index.assign(N_SCAN, 0);
        cloudInfo.point_col_ind.assign(N_SCAN*Horizon_SCAN, 0);
        cloudInfo.point_range.assign(N_SCAN*Horizon_SCAN, 0);
        
        resetParameters();
    }

    void resetParameters()
    {
        
        laserCloudIn->clear();
        extractedCloud->clear();
        
        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
       
        imuPointerCur = 0;
        firstPointFlag = true;
        odomDeskewFlag = false;
        
        for (int i = 0; i < queueLength; ++i)
        {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
        }
    }
    

    ~ImageProjection(){}

    void imuHandler(const sensor_msgs::msg::Imu::ConstPtr & imuMsg)
    {
        sensor_msgs::msg::Imu thisImu = imuConverter(*imuMsg);

        std::lock_guard<std::mutex> lock1(imuLock);

        imuQueue.push_back(thisImu);
    }

    void odometryHandler(const nav_msgs::msg::Odometry::ConstPtr & odometryMsg)
    {
        std::lock_guard<std::mutex> lock2(odoLock);

        odomQueue.push_back(*odometryMsg);
    }

    void cloudHandler(const sensor_msgs::msg::PointCloud2::ConstPtr& laserCloudMsg)
    {
        if (!cachePointCloud(laserCloudMsg))
            return;

        if (!deskewInfo())
            return;

        projectPointCloud();

        cloudExtraction();

        publishCloud();

        resetParameters();
    }

    bool cachePointCloud(const sensor_msgs::msg::PointCloud2::ConstPtr& laserCloudMsg)
    {
        cloudQueue.push_back(*laserCloudMsg);

        if (cloudQueue.size() <= 2)
            return false;

        currentCloudMsg = cloudQueue.front();

        cloudQueue.pop_front();

        pcl::fromROSMsg(currentCloudMsg, *laserCloudIn);

        cloudHeader = currentCloudMsg.header;
        timeScanCur = cloudHeader.stamp.sec + cloudHeader.stamp.nanosec * 1e-9;
        timeScanNext = cloudQueue.front().header.stamp.sec + cloudQueue.front().header.stamp.nanosec * 1e-9;

        if (laserCloudIn->is_dense == false)
        {
            RCLCPP_ERROR(get_logger(), "Point cloud is not in dense format, please remove NaN points first!");
            rclcpp::shutdown();
        }

        static int ringFlag = 0;
        if (ringFlag == 0)
        {
            ringFlag = -1;
            for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
            {
                if (currentCloudMsg.fields[i].name == "ring")
                {
                    ringFlag = 1;
                    break;
                }
            }
            if (ringFlag == -1)
            {
                RCLCPP_ERROR(get_logger(), "Point cloud ring channel not available, please configure your point cloud data!");
                rclcpp::shutdown();
            }
        }   

        if (deskewFlag == 0)
        {
            deskewFlag = -1;
            for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
            {
                if (currentCloudMsg.fields[i].name == "time")
                {
                    deskewFlag = 1;
                    break;
                }
            }
            if (deskewFlag == -1)
                RCLCPP_WARN(get_logger(), "Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
        }

        return true;
    }

    bool deskewInfo()
    {
        std::lock_guard<std::mutex> lock1(imuLock);
        std::lock_guard<std::mutex> lock2(odoLock);
        
        auto t_f = imuQueue.front().header.stamp.sec + imuQueue.front().header.stamp.nanosec * 1e-9;
        auto t_b = imuQueue.back().header.stamp.sec + imuQueue.back().header.stamp.nanosec * 1e-9;
        
        if (imuQueue.empty() || t_f > timeScanCur || t_b < timeScanNext)
        {
            RCLCPP_DEBUG(get_logger(), "Waiting for IMU data ...");
            return false;
        }
        
        imuDeskewInfo();
        
        odomDeskewInfo();
        
        return true;
    }

    void imuDeskewInfo()
    {
        cloudInfo.imu_available = false;
        
        while (!imuQueue.empty())
        {
            auto t_f = imuQueue.front().header.stamp.sec +
        imuQueue.front().header.stamp.nanosec * 1e-9;
            if (t_f < timeScanCur - 0.01)
                imuQueue.pop_front();
            else
                break;
        }
        
        if (imuQueue.empty())
            return;
        
        imuPointerCur = 0;
        
        for (int i = 0; i < (int)imuQueue.size(); ++i)
        {
            sensor_msgs::msg::Imu thisImuMsg = imuQueue[i];

            double currentImuTime = thisImuMsg.header.stamp.sec +
        thisImuMsg.header.stamp.nanosec * 1e-9;
            
            if (currentImuTime > timeScanNext + 0.01)
                break;
                
            if (imuPointerCur == 0){
                imuRotX[0] = 0;
                imuRotY[0] = 0;
                imuRotZ[0] = 0;
                imuTime[0] = currentImuTime;
                ++imuPointerCur;
                continue;
            }

            double angular_x, angular_y, angular_z;
            imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

            double timeDiff = currentImuTime - imuTime[imuPointerCur-1];
            imuRotX[imuPointerCur] = imuRotX[imuPointerCur-1] + angular_x * timeDiff;
            imuRotY[imuPointerCur] = imuRotY[imuPointerCur-1] + angular_y * timeDiff;
            imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur-1] + angular_z * timeDiff;
            imuTime[imuPointerCur] = currentImuTime;
            ++imuPointerCur;
        }
        --imuPointerCur;
        
        if (imuPointerCur <= 0)
            return;

        cloudInfo.imu_available = true;
    }

    void odomDeskewInfo()
    {
        cloudInfo.odom_available = false;
        
        while (!odomQueue.empty())
        {
            auto time = odomQueue.front().header.stamp.sec + odomQueue.front().header.stamp.nanosec * 1e-9;
            if (time < timeScanCur - 0.01)
                odomQueue.pop_front();
            else
                break;
        }
        
        if (odomQueue.empty())
            return;
        
        auto t_f = odomQueue.front().header.stamp.sec + odomQueue.front().header.stamp.nanosec * 1e-9;    
        if (t_f > timeScanCur)
            return;

        nav_msgs::msg::Odometry startOdomMsg;
        for (int i = 0; i < odomQueue.size(); ++i)
        {
            startOdomMsg = odomQueue[i];
            
            if (ROS_TIME(&startOdomMsg) < timeScanCur)
                continue;
            else
                break;
        }

        tf2::Quaternion orientation;

        tf2::fromMsg(startOdomMsg.pose.pose.orientation, orientation);

        double roll, pitch, yaw;
        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        cloudInfo.initial_guess_x = startOdomMsg.pose.pose.position.x;
        cloudInfo.initial_guess_y = startOdomMsg.pose.pose.position.y;
        cloudInfo.initial_guess_z = startOdomMsg.pose.pose.position.z;
        cloudInfo.initial_guess_roll = roll;
        cloudInfo.initial_guess_pitch = pitch;
        cloudInfo.initial_guess_yaw = yaw;

        cloudInfo.odom_available = true;

        odomDeskewFlag = false;
        
        auto t_b = odomQueue.back().header.stamp.sec + odomQueue.back().header.stamp.nanosec * 1e-9;
        if (t_b  < timeScanNext)
            return;
            
        nav_msgs::msg::Odometry endOdomMsg;
        
        for (int i = 0; i < odomQueue.size(); ++i)
        {
            endOdomMsg = odomQueue[i];

            if (ROS_TIME(&endOdomMsg) < timeScanNext)
                continue;
            else
                break;
        }

        if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))
            return;

        Eigen::Affine3f transBegin = pcl::getTransformation(startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y, startOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        tf2::fromMsg(endOdomMsg.pose.pose.orientation, orientation);
        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        Eigen::Affine3f transEnd = pcl::getTransformation(endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y, endOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

        float rollIncre, pitchIncre, yawIncre;
        pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);

        odomDeskewFlag = true;
    }

    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
    {

        *rotXCur = 0; *rotYCur = 0; *rotZCur = 0;

        int imuPointerFront = 0;
        while (imuPointerFront < imuPointerCur)
        {
            if (pointTime < imuTime[imuPointerFront])
                break;
            ++imuPointerFront;
        }

        if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0)
        {
            *rotXCur = imuRotX[imuPointerFront];
            *rotYCur = imuRotY[imuPointerFront];
            *rotZCur = imuRotZ[imuPointerFront];
        }
        else
        {
            int imuPointerBack = imuPointerFront - 1;
            double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
            *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
            *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
        }
    }

    void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur)
    {
        *posXCur = 0; *posYCur = 0; *posZCur = 0;
    }

    PointType deskewPoint(PointType *point, double relTime)
    {
        if (deskewFlag == -1 || cloudInfo.imu_available == false)
            return *point;

        double pointTime = timeScanCur + relTime;

        float rotXCur, rotYCur, rotZCur;

        findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

        float posXCur, posYCur, posZCur;

        findPosition(relTime, &posXCur, &posYCur, &posZCur);

        if (firstPointFlag == true)
        {
            transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
            firstPointFlag = false;
        }

        Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
        Eigen::Affine3f transBt = transStartInverse * transFinal;

        PointType newPoint;
        newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);
        newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);
        newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);
        newPoint.intensity = point->intensity;

        return newPoint;
    }


    void projectPointCloud()
    {
        int cloudSize = laserCloudIn->points.size();

        for (int i = 0; i < cloudSize; ++i)
        {
            PointType thisPoint;
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            thisPoint.intensity = laserCloudIn->points[i].intensity;

            float range = pointDistance(thisPoint);
            if (range < lidarMinRange || range > lidarMaxRange)
                continue;

            int rowIdn = laserCloudIn->points[i].ring;
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;
            if (rowIdn % downsampleRate != 0)
                continue;

            float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
            static float ang_res_x = 360.0/float(Horizon_SCAN);
            int columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
            
            if (columnIdn >= Horizon_SCAN)
                columnIdn -= Horizon_SCAN;
                
            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;
                
            if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
                continue;

            thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);

            rangeMat.at<float>(rowIdn, columnIdn) = range;

            int index = columnIdn + rowIdn * Horizon_SCAN;

            fullCloud->points[index] = thisPoint;
        }
    }

    void cloudExtraction()
    {
        int count = 0;
        
        for (int i = 0; i < N_SCAN; ++i)
        {
            cloudInfo.start_ring_index[i] = count - 1 + 5;
            for (int j = 0; j < Horizon_SCAN; ++j)
            {
                if (rangeMat.at<float>(i,j) != FLT_MAX)
                {
                    cloudInfo.point_col_ind[count] = j;
                    cloudInfo.point_range[count] = rangeMat.at<float>(i,j);
                    extractedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                    ++count;
                }
            }
            cloudInfo.end_ring_index[i] = count -1 - 5;
        }
    }
    
    void publishCloud()
    {
        sensor_msgs::msg::PointCloud2 tempCloud;
        pcl::toROSMsg(*extractedCloud, tempCloud);
        tempCloud.header.stamp = cloudHeader.stamp;
        tempCloud.header.frame_id = "base_link";
        pubLaserCloudInfo->publish(tempCloud);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    rclcpp::executors::SingleThreadedExecutor exec;

    auto ip = std::make_shared<ImageProjection>(options);
    exec.add_node(ip);
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Image Projection Started.\033[0m");
    
    exec.spin();
    rclcpp::shutdown();
    
    return 0;
}
