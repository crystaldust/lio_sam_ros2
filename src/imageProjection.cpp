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
)  // 不懂这种用法

// Ouster
// struct PointXYZIRT {
//     PCL_ADD_POINT4D;
//     float intensity;
//     uint32_t t;
//     uint16_t reflectivity;
//     uint8_t ring;
//     uint16_t noise;
//     uint32_t range;
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// }EIGEN_ALIGN16;

// POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
//     (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
//     (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
//     (uint8_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range)
// )

const int queueLength = 2000;

class ImageProjection : public ParamServer
{
private:

    std::mutex imuLock;
    std::mutex odoLock;

    //- rclcpp::Subscriber subLaserCloud;
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

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;  // 雷达直接传出的点云
    pcl::PointCloud<PointType>::Ptr   fullCloud;  // 投影后的点云
    pcl::PointCloud<PointType>::Ptr   extractedCloud;  // 

    int deskewFlag;
    cv::Mat rangeMat;

    bool odomDeskewFlag;
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;

    lio_sam_ros2::msg::CloudInfo cloudInfo;
    double timeScanCur;
    double timeScanEnd;
    std_msgs::msg::Header cloudHeader;


public:
    ImageProjection(const rclcpp::NodeOptions & options): ParamServer("lio_sam_ros2_imageProjection", options), deskewFlag(0)
    {
        // 订阅IMU话题
        auto imu_callback = [this](const sensor_msgs::msg::Imu::ConstPtr msg) -> void
        {
            imuHandler(msg);
        };
        subImu = create_subscription<sensor_msgs::msg::Imu>(imuTopic, rclcpp::SensorDataQoS(), imu_callback);
        // 订阅地图优化程序中发布的里程计话题
        auto odom_callback = [this](const nav_msgs::msg::Odometry::ConstPtr msg) -> void
        {
            odometryHandler(msg);
        };
        subOdom = create_subscription<nav_msgs::msg::Odometry>(odomTopic, rclcpp::SensorDataQoS(), odom_callback);
        // 订阅原始激光点云
        auto lc_callback = [this](const sensor_msgs::msg::PointCloud2::ConstPtr msg) -> void
        {
            cloudHandler(msg);
        };
        subLaserCloud = create_subscription<sensor_msgs::msg::PointCloud2>(pointCloudTopic, 5, lc_callback);
        // 发布去畸变的点云
        pubExtractedCloud = create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam_ros2/deskew/cloud_deskewed", 2000);
        // 发布原始激光点云信息
        pubLaserCloudInfo = create_publisher<sensor_msgs::msg::PointCloud2>("lio_sam_ros2/deskew/cloud_info", 2000);
        // 分配内存
        allocateMemory();
        // 重置部分参数
        resetParameters();
        // 某种报错机制，不懂
        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }

    void allocateMemory()
    {   
        // 给原始点云分配内存
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        // 给投影后的点云分配内存
        fullCloud.reset(new pcl::PointCloud<PointType>());
        // 
        extractedCloud.reset(new pcl::PointCloud<PointType>());
        // fullCloud中的点云进行维度转换
        fullCloud->points.resize(N_SCAN*Horizon_SCAN);
        // 下面四行暂时弄不懂
        cloudInfo.start_ring_index.assign(N_SCAN, 0);
        cloudInfo.end_ring_index.assign(N_SCAN, 0);
        cloudInfo.point_col_ind.assign(N_SCAN*Horizon_SCAN, 0);
        cloudInfo.point_range.assign(N_SCAN*Horizon_SCAN, 0);
        // 重置部分参数
        resetParameters();
    }

    void resetParameters()
    {
        // 清零操作
        laserCloudIn->clear();
        extractedCloud->clear();
        // reset range matrix for range image projection
        // 下面这行暂时不懂
        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
        // 暂时不知道这些变量的意思
        imuPointerCur = 0;
        firstPointFlag = true;
        odomDeskewFlag = false;
        // 重置imu队列中imu数据
        for (int i = 0; i < queueLength; ++i)
        {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
        }
    }
    
    // 析构函数
    ~ImageProjection(){}

    // 回调函数
    void imuHandler(const sensor_msgs::msg::Imu::ConstPtr & imuMsg)
    {
        // 将原始IMU数据从IMU坐标系转换到雷达坐标系
        sensor_msgs::msg::Imu thisImu = imuConverter(*imuMsg);
        // 锁存操作，保证数据一次性处理完成
        std::lock_guard<std::mutex> lock1(imuLock);
        // 将IMU数据存放到队列中
        imuQueue.push_back(thisImu);
        // 下面是调试IMU用的
        // debug IMU data
        // cout << std::setprecision(6);
        // cout << "IMU acc: " << endl;
        // cout << "x: " << thisImu.linear_acceleration.x << 
        //       ", y: " << thisImu.linear_acceleration.y << 
        //       ", z: " << thisImu.linear_acceleration.z << endl;
        // cout << "IMU gyro: " << endl;
        // cout << "x: " << thisImu.angular_velocity.x << 
        //       ", y: " << thisImu.angular_velocity.y << 
        //       ", z: " << thisImu.angular_velocity.z << endl;
        // double imuRoll, imuPitch, imuYaw;
        // tf::Quaternion orientation;
        // tf::quaternionMsgToTF(thisImu.orientation, orientation);
        // tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);
        // cout << "IMU roll pitch yaw: " << endl;
        // cout << "roll: " << imuRoll << ", pitch: " << imuPitch << ", yaw: " << imuYaw << endl << endl;
    }

    // 回调函数
    void odometryHandler(const nav_msgs::msg::Odometry::ConstPtr & odometryMsg)
    {
        // 同样的索存操作
        std::lock_guard<std::mutex> lock2(odoLock);
        // 同样存放到队列中
        odomQueue.push_back(*odometryMsg);
    }

    // 回调函数
    // 进行点云去畸变和投影操作
    void cloudHandler(const sensor_msgs::msg::PointCloud2::ConstPtr& laserCloudMsg)
    {
        // 缓存点云信息
        // 学习该写法
        if (!cachePointCloud(laserCloudMsg))
            return;
        // 计算去畸变的参数
        if (!deskewInfo())
            return;
        // 投影
        projectPointCloud();
        // 点云提取
        cloudExtraction();
        // 发布点云
        publishClouds();
        // 重置参数
        resetParameters();
    }

    // 缓存点云信息
    bool cachePointCloud(const sensor_msgs::msg::PointCloud2::ConstPtr& laserCloudMsg)
    {
        // cache point cloud
        // 存入队列中
        // cloudQueue的类型 std::deque<sensor_msgs::PointCloud2>
        cloudQueue.push_back(*laserCloudMsg);
        // 点云太少就退出
        if (cloudQueue.size() <= 2)
            return false;
        // convert cloud
        // 存到currentCloudMsg的类型是sensor_msgs::PointCloud2
        currentCloudMsg = cloudQueue.front();
        // 推出该数据，因为上一行已经存好
        cloudQueue.pop_front();
        // 从ROS message转化为点云
        pcl::fromROSMsg(currentCloudMsg, *laserCloudIn);
        // get timestamp
        // 获取头和时间
        // 由于进行了pop操作，队列顶的元素是下一帧的时间
        cloudHeader = currentCloudMsg.header;
        timeScanCur = cloudHeader.stamp.sec;
        timeScanEnd = timeScanCur + laserCloudIn->points.back().time; // Velodyne
        // timeScanEnd = timeScanCur + (float)laserCloudIn->points.back().t / 1000000000.0; // Ouster
        // check dense flag
        // 检查点云是否有无效点
        if (laserCloudIn->is_dense == false)
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Point cloud is not in dense format, please remove NaN points first!");
            rclcpp::shutdown();
        }
        // check ring channel
        // 查看点云中ring字段是否存在
        // velodyne和Ouster中都有该字段
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
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Point cloud ring channel not available, please configure your point cloud data!");
                rclcpp::shutdown();
            }
        }   
        // check point time
        // 检查time字段，去畸变时起作用
        if (deskewFlag == 0)
        {
            deskewFlag = -1;
            for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
            {
                if (currentCloudMsg.fields[i].name == timeField)
                {
                    deskewFlag = 1;
                    break;
                }
            }
            if (deskewFlag == -1)
                RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
        }

        return true;
    }

    // 去畸变的相关关键信息或变量值计算
    bool deskewInfo()
    {
        // 锁存，确保数据都存进来了
        std::lock_guard<std::mutex> lock1(imuLock);
        std::lock_guard<std::mutex> lock2(odoLock);
        // make sure IMU data available for the scan
        // 这里判断条件很多，作者是为了确定需要高频IMU数据
        // 作者使用500Hz的，100Hz的也可以
        // 判断条件：IMU队列为空不行，IMU队列顶的元素时间应该小于当前激光扫描时间，队列底的元素时间应该大于下一帧的时间戳
        if (imuQueue.empty() || imuQueue.front().header.stamp.sec > timeScanCur || imuQueue.back().header.stamp.sec < timeScanEnd)
        {
            RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Waiting for IMU data ...");
            return false;
        }
        // IMU去畸变参数计算
        imuDeskewInfo();
        // odom去畸变参数计算
        odomDeskewInfo();
        return true;
    }

    void imuDeskewInfo()
    {
        // 该参数在mapOptmization程序中使用
        // 首先为false，完成相关操作后置true
        cloudInfo.imu_available = false;
        while (!imuQueue.empty())
        {
            // 继续舍弃比较旧的IMU数据，以0.01为阈值
            if (imuQueue.front().header.stamp.sec < timeScanCur - 0.01)
                imuQueue.pop_front();
            else
                break;
        }
        if (imuQueue.empty())
            return;
        imuPointerCur = 0;
        for (int i = 0; i < (int)imuQueue.size(); ++i)
        {
            // 去除队列中的元素
            sensor_msgs::msg::Imu thisImuMsg = imuQueue[i];
            // 获取当前帧的时间戳
            double currentImuTime = thisImuMsg.header.stamp.sec;
            // get roll, pitch, and yaw estimation for this scan
            // 用IMU的欧拉角做扫描的位姿估计，直接把值赋给cloudInfo
            // 如果当前IMU数据符合条件，得到相关的信息
            if (currentImuTime <= timeScanCur)
                imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imu_roll_init, &cloudInfo.imu_pitch_init, &cloudInfo.imu_yaw_init);
            // 不符合的话直接退出
            if (currentImuTime > timeScanEnd + 0.01)
                break;
            // 初始化操作
            if (imuPointerCur == 0){
                imuRotX[0] = 0;
                imuRotY[0] = 0;
                imuRotZ[0] = 0;
                imuTime[0] = currentImuTime;
                ++imuPointerCur;
                continue;
            }
            // get angular velocity
            // 获得角速度
            double angular_x, angular_y, angular_z;
            imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);
            // integrate rotation
            // 把角速度和时间间隔积分，得到转角，用于之后的去畸变
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
        // 接开始，将该变量置true，用于之后的地图优化
        cloudInfo.imu_available = true;
    }

    void odomDeskewInfo()
    {
        // 类似于IMU去畸变中的变量
        cloudInfo.odom_available = false;
        while (!odomQueue.empty())
        {
            if (odomQueue.front().header.stamp.sec < timeScanCur - 0.01)
                odomQueue.pop_front();
            else
                break;
        }
        if (odomQueue.empty())
            return;
        if (odomQueue.front().header.stamp.sec > timeScanCur)
            return;
        // get start odometry at the beinning of the scan
        // 获得每一帧起始时刻的里程计信息
        nav_msgs::msg::Odometry startOdomMsg;
        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            startOdomMsg = odomQueue[i];
            if (ROS_TIME(&startOdomMsg) < timeScanCur)
                continue;
            else
                break;
        }
        // 方向，是以四元数存储的
        tf2::Quaternion orientation;
        // 方向转化为TF坐标?
        tf2::fromMsg(startOdomMsg.pose.pose.orientation, orientation);
        // 从方向中获得旋转角
        double roll, pitch, yaw;
        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        // Initial guess used in mapOptimization
        // 初始化位姿估计，在地图优化中使用
        cloudInfo.initial_guess_x = startOdomMsg.pose.pose.position.x;
        cloudInfo.initial_guess_y = startOdomMsg.pose.pose.position.y;
        cloudInfo.initial_guess_z = startOdomMsg.pose.pose.position.z;
        cloudInfo.initial_guess_roll = roll;
        cloudInfo.initial_guess_pitch = pitch;
        cloudInfo.initial_guess_yaw = yaw;
        // 标志变量置true
        cloudInfo.odom_available = true;
        // get end odometry at the end of the scan
        // 获得一帧扫描末尾的里程计信息，用于去畸变
        odomDeskewFlag = false;
        if (odomQueue.back().header.stamp.sec < timeScanEnd)
            return;
        nav_msgs::msg::Odometry endOdomMsg;
        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            endOdomMsg = odomQueue[i];

            if (ROS_TIME(&endOdomMsg) < timeScanEnd)
                continue;
            else
                break;
        }
        // 位姿协方差矩阵计算，确保是同一帧
        if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))
            return;
        // 获得起始的变换
        Eigen::Affine3f transBegin = pcl::getTransformation(startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y, startOdomMsg.pose.pose.position.z, roll, pitch, yaw);
        // 获得结束的变换
        tf2::fromMsg(endOdomMsg.pose.pose.orientation, orientation);
        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        Eigen::Affine3f transEnd = pcl::getTransformation(endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y, endOdomMsg.pose.pose.position.z, roll, pitch, yaw);
        // 获得一帧扫描起始和结束时刻之间的变换，loam中介绍了该处
        Eigen::Affine3f transBt = transBegin.inverse() * transEnd;
        // 获得欧拉角增量值
        float rollIncre, pitchIncre, yawIncre;
        pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);
        // 标志变量置true
        odomDeskewFlag = true;
    }

    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
    {
        // 得到旋转量
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
            // 根据点的时间信息，获得每个点的时刻的旋转变化量
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
        // 对于高速移动的过程，下面的注释部分有用
        // 对于低速运动的过程，里程计增量设置为0即可。
        *posXCur = 0; *posYCur = 0; *posZCur = 0;

        // If the sensor moves relatively slow, like walking speed, positional deskew seems to have little benefits. Thus code below is commented.

        // if (cloudInfo.odomAvailable == false || odomDeskewFlag == false)
        //     return;

        // float ratio = relTime / (timeScanEnd - timeScanCur);

        // *posXCur = ratio * odomIncreX;
        // *posYCur = ratio * odomIncreY;
        // *posZCur = ratio * odomIncreZ;
    }

    PointType deskewPoint(PointType *point, double relTime)
    {
        // 运动补偿函数
        // 这是调用函数的语句 thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time); // velodyne
        if (deskewFlag == -1 || cloudInfo.imu_available == false)
            return *point;
        // 点在一帧中的具体时间
        double pointTime = timeScanCur + relTime;
        // 定义相关用于旋转的变量
        float rotXCur, rotYCur, rotZCur;
        // 该函数获得相关旋转量
        findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);
        // 定义相关用于平移的变量
        float posXCur, posYCur, posZCur;
        // 该函数获得相关平移量
        findPosition(relTime, &posXCur, &posYCur, &posZCur);
        // 如果第一次收到数据
        if (firstPointFlag == true)
        {
            // 起始变换矩阵赋初值再取逆
            transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
            firstPointFlag = false;
        }
        // transform points to start
        // 把点投影到每一帧扫描的起始时刻，参考loam的解决方案
        Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
        Eigen::Affine3f transBt = transStartInverse * transFinal;
        // 得到去畸变的点云
        PointType newPoint;
        newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);
        newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);
        newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);
        newPoint.intensity = point->intensity;

        return newPoint;
    }

    // 投影点云
    void projectPointCloud()
    {
        int cloudSize = laserCloudIn->points.size();  // 所有雷达传出的点云
        // 点云按线束，行列保存
        for (int i = 0; i < cloudSize; ++i)
        {
            PointType thisPoint;
            // 原始点云中的数据
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            thisPoint.intensity = laserCloudIn->points[i].intensity;
            // 
            float range = pointDistance(thisPoint);
            // 
            if (range < lidarMinRange || range > lidarMaxRange)
                continue;
            // 某个点属于哪个圆，即属于雷达的哪根线
            // 根据线过滤点云
            int rowIdn = laserCloudIn->points[i].ring;
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;
            if (rowIdn % downsampleRate != 0)
                continue;
            // 水平角
            float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
            // 角分辨率，360/1800
            static float ang_res_x = 360.0/float(Horizon_SCAN);
            // 计算距离图像上点属于那一列
            int columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
            if (columnIdn >= Horizon_SCAN)
                columnIdn -= Horizon_SCAN;
            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;
            if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
                continue;
            // 进行点云矫正畸变
            thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time); // Velodyne
            // thisPoint = deskewPoint(&thisPoint, (float)laserCloudIn->points[i].t / 1000000000.0); // Ouster
            rangeMat.at<float>(rowIdn, columnIdn) = range;
            // 计算索引
            int index = columnIdn + rowIdn * Horizon_SCAN;
            // 将滤波后的点云投影后存储到fullCloud
            fullCloud->points[index] = thisPoint;
        }
    }

    void cloudExtraction()
    {
        // 提取点云函数
        int count = 0;
        // extract segmented cloud for lidar odometry
        for (int i = 0; i < N_SCAN; ++i)
        {
            cloudInfo.start_ring_index[i] = count - 1 + 5;
            for (int j = 0; j < Horizon_SCAN; ++j)
            {
                if (rangeMat.at<float>(i,j) != FLT_MAX)
                {
                    // mark the points' column index for marking occlusion later
                    cloudInfo.point_col_ind[count] = j;
                    // save range info
                    cloudInfo.point_range[count] = rangeMat.at<float>(i,j);
                    // save extracted cloud
                    extractedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                    // size of extracted cloud
                    ++count;
                }
            }
            cloudInfo.end_ring_index[i] = count -1 - 5;
        }
    }
    
    void publishClouds()
    {
        // 发布点云函数
        cloudInfo.header = cloudHeader;
        cloudInfo.cloud_deskewed  = publishCloud(&pubExtractedCloud, extractedCloud, cloudHeader.stamp, lidarFrame);
        pubLaserCloudInfo->publish(cloudInfo);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv, "lio_sam_ros2");
    ImageProjection IP; 
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Image Projection Started.\033[0m");
    rclcpp::MultiThreadedSpinner spinner(3);
    spinner.spin();
    return 0;
}
