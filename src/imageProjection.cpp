#include "utility.h"
#include "liorf/msg/cloud_info.hpp"
// <!-- liorf_localization_yjz_lucky_boy -->
struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
    (uint8_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range)
)

// Recent ouster_ros releases expose `ring` as uint8, while the RESPLE
// TestTrack recording uses the PointCloud2 convention's uint16 ring field.
// PCL requires an exact datatype match when mapping named fields, so retain
// both layouts instead of silently decoding every uint16 ring as zero.
struct OusterPointXYZIRT16 {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint16_t ring;
    uint16_t ambient;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT16,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
    (uint16_t, ring, ring) (uint16_t, ambient, ambient) (uint32_t, range, range)
)

// The workspace's common Livox bridge preserves the native relative timestamp
// as uint32 nanoseconds and the Livox line as a uint16 ring. This differs from
// both the Velodyne float-seconds layout and Ouster's uint8 ring layout.
struct LivoxPointXYZIRT {
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    uint32_t t;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(LivoxPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint32_t, t, t) (uint16_t, ring, ring)
)

struct RobosensePointXYZIRT
{
    PCL_ADD_POINT4D
    float intensity;
    uint16_t ring;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(RobosensePointXYZIRT,
      (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
      (uint16_t, ring, ring)(double, timestamp, timestamp)
)

// mulran datasets
struct MulranPointXYZIRT {
    PCL_ADD_POINT4D
    float intensity;
    uint32_t t;
    int ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 }EIGEN_ALIGN16;
 POINT_CLOUD_REGISTER_POINT_STRUCT (MulranPointXYZIRT,
     (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
     (uint32_t, t, t) (int, ring, ring)
 )

// Use the Velodyne point format as a common representation
using PointXYZIRT = VelodynePointXYZIRT;

const int queueLength = 2000;

class ImageProjection : public ParamServer
{
private:

    std::mutex imuLock;
    std::mutex odoLock;

    // Mutually exclusive groups: messages within each group stay ordered, which
    // the IMU/odometry queues depend on. Keeping the cloud in its own group
    // preserves the ROS 1 behaviour where IMU kept arriving during a deskew.
    rclcpp::CallbackGroup::SharedPtr sensorCallbackGroup;
    rclcpp::CallbackGroup::SharedPtr cloudCallbackGroup;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubExtractedCloud;
    rclcpp::Publisher<liorf::msg::CloudInfo>::SharedPtr pubLaserCloudInfo;

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
    pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;
    pcl::PointCloud<OusterPointXYZIRT16>::Ptr tmpOusterCloudIn16;
    pcl::PointCloud<LivoxPointXYZIRT>::Ptr tmpLivoxCloudIn;
    pcl::PointCloud<MulranPointXYZIRT>::Ptr tmpMulranCloudIn;
    pcl::PointCloud<PointType>::Ptr   fullCloud;

    int deskewFlag;

    bool odomDeskewFlag;
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;

    liorf::msg::CloudInfo cloudInfo;
    double timeScanCur;
    double timeScanEnd;
    std_msgs::msg::Header cloudHeader;

public:
    explicit ImageProjection(const rclcpp::NodeOptions & options):
    ParamServer("liorf_imageProjection", options),
    deskewFlag(0)
    {
        sensorCallbackGroup = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cloudCallbackGroup  = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto sensorOpt = rclcpp::SubscriptionOptions();
        sensorOpt.callback_group = sensorCallbackGroup;
        auto cloudOpt = rclcpp::SubscriptionOptions();
        cloudOpt.callback_group = cloudCallbackGroup;

        subImu        = create_subscription<sensor_msgs::msg::Imu>(
            prefixTopic(robot_id, imuTopic), rclcpp::SensorDataQoS().keep_last(2000),
            std::bind(&ImageProjection::imuHandler, this, std::placeholders::_1), sensorOpt);
        subOdom       = create_subscription<nav_msgs::msg::Odometry>(
            prefixTopic(robot_id, odomTopic + "_incremental"), rclcpp::QoS(2000),
            std::bind(&ImageProjection::odometryHandler, this, std::placeholders::_1), sensorOpt);
        subLaserCloud = create_subscription<sensor_msgs::msg::PointCloud2>(
            prefixTopic(robot_id, pointCloudTopic), rclcpp::SensorDataQoS().keep_last(5),
            std::bind(&ImageProjection::cloudHandler, this, std::placeholders::_1), cloudOpt);

        pubExtractedCloud = create_publisher<sensor_msgs::msg::PointCloud2>(
            robot_id + "/liorf/deskew/cloud_deskewed", 1);
        pubLaserCloudInfo = create_publisher<liorf::msg::CloudInfo>(
            robot_id + "/liorf/deskew/cloud_info", 1);

        allocateMemory();
        resetParameters();

        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }

    void allocateMemory()
    {
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        tmpOusterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());
        tmpOusterCloudIn16.reset(new pcl::PointCloud<OusterPointXYZIRT16>());
        tmpLivoxCloudIn.reset(new pcl::PointCloud<LivoxPointXYZIRT>());
        tmpMulranCloudIn.reset(new pcl::PointCloud<MulranPointXYZIRT>());
        fullCloud.reset(new pcl::PointCloud<PointType>());

        resetParameters();
    }

    void resetParameters()
    {
        laserCloudIn->clear();
        fullCloud->clear();

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

    ~ImageProjection()
    {
        delete[] imuTime;
        delete[] imuRotX;
        delete[] imuRotY;
        delete[] imuRotZ;
    }

    void imuHandler(const sensor_msgs::msg::Imu::SharedPtr imuMsg)
    {
        sensor_msgs::msg::Imu thisImu = imuConverter(*imuMsg);

        std::lock_guard<std::mutex> lock1(imuLock);
        imuQueue.push_back(thisImu);
    }

    void odometryHandler(const nav_msgs::msg::Odometry::SharedPtr odometryMsg)
    {
        std::lock_guard<std::mutex> lock2(odoLock);
        odomQueue.push_back(*odometryMsg);
    }

    void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg)
    {
        if (!cachePointCloud(laserCloudMsg))
            return;

        if (!deskewInfo())
            return;

        projectPointCloud();

        publishClouds();

        resetParameters();
    }

    bool cachePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& laserCloudMsg)
    {
        // cache point cloud
        cloudQueue.push_back(*laserCloudMsg);
        if (cloudQueue.size() <= 2)
            return false;

        // convert cloud
        currentCloudMsg = std::move(cloudQueue.front());
        cloudQueue.pop_front();
        if (sensor == SensorType::VELODYNE)
        {
            pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);
        }
        else if (sensor == SensorType::LIVOX)
        {
            // Native Livox PointCloud2 producers commonly expose float
            // seconds as `time`; the benchmark bridge exposes uint32
            // nanoseconds as `t`. Support both without silently zeroing the
            // deskew time through a mismatched PCL field conversion.
            bool hasNanosecondTime = false;
            for (const auto &field : currentCloudMsg.fields)
            {
                if (field.name == "t" &&
                    field.datatype == sensor_msgs::msg::PointField::UINT32)
                {
                    hasNanosecondTime = true;
                    break;
                }
            }

            if (!hasNanosecondTime)
            {
                pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);
            }
            else
            {
                pcl::moveFromROSMsg(currentCloudMsg, *tmpLivoxCloudIn);
                laserCloudIn->points.resize(tmpLivoxCloudIn->size());
                laserCloudIn->is_dense = tmpLivoxCloudIn->is_dense;
                for (size_t i = 0; i < tmpLivoxCloudIn->size(); i++)
                {
                    auto &src = tmpLivoxCloudIn->points[i];
                    auto &dst = laserCloudIn->points[i];
                    dst.x = src.x;
                    dst.y = src.y;
                    dst.z = src.z;
                    dst.intensity = src.intensity;
                    dst.ring = src.ring;
                    dst.time = src.t * 1e-9f;
                }
            }
        }
        else if (sensor == SensorType::OUSTER)
        {
            const auto ringField = std::find_if(
                currentCloudMsg.fields.begin(), currentCloudMsg.fields.end(),
                [](const sensor_msgs::msg::PointField &field) {
                    return field.name == "ring";
                });
            if (ringField == currentCloudMsg.fields.end())
            {
                RCLCPP_ERROR(get_logger(),
                    "Ouster point cloud ring channel not available");
                rclcpp::shutdown();
                return false;
            }

            if (ringField->datatype == sensor_msgs::msg::PointField::UINT8)
            {
                pcl::moveFromROSMsg(currentCloudMsg, *tmpOusterCloudIn);
                laserCloudIn->points.resize(tmpOusterCloudIn->size());
                laserCloudIn->is_dense = tmpOusterCloudIn->is_dense;
                for (size_t i = 0; i < tmpOusterCloudIn->size(); i++)
                {
                    const auto &src = tmpOusterCloudIn->points[i];
                    auto &dst = laserCloudIn->points[i];
                    dst.x = src.x;
                    dst.y = src.y;
                    dst.z = src.z;
                    dst.intensity = src.intensity;
                    dst.ring = src.ring;
                    dst.time = src.t * 1e-9f;
                }
            }
            else if (ringField->datatype == sensor_msgs::msg::PointField::UINT16)
            {
                pcl::moveFromROSMsg(currentCloudMsg, *tmpOusterCloudIn16);
                laserCloudIn->points.resize(tmpOusterCloudIn16->size());
                laserCloudIn->is_dense = tmpOusterCloudIn16->is_dense;
                for (size_t i = 0; i < tmpOusterCloudIn16->size(); i++)
                {
                    const auto &src = tmpOusterCloudIn16->points[i];
                    auto &dst = laserCloudIn->points[i];
                    dst.x = src.x;
                    dst.y = src.y;
                    dst.z = src.z;
                    dst.intensity = src.intensity;
                    dst.ring = src.ring;
                    dst.time = src.t * 1e-9f;
                }
            }
            else
            {
                RCLCPP_ERROR(get_logger(),
                    "Unsupported Ouster ring datatype %u (expected UINT8 or UINT16)",
                    static_cast<unsigned>(ringField->datatype));
                rclcpp::shutdown();
                return false;
            }
        } // <!-- liorf_yjz_lucky_boy -->
        else if (sensor == SensorType::MULRAN)
        {
            // Convert to Velodyne format
            pcl::moveFromROSMsg(currentCloudMsg, *tmpMulranCloudIn);
            laserCloudIn->points.resize(tmpMulranCloudIn->size());
            laserCloudIn->is_dense = tmpMulranCloudIn->is_dense;
            for (size_t i = 0; i < tmpMulranCloudIn->size(); i++)
            {
                auto &src = tmpMulranCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                dst.time = static_cast<float>(src.t);
            }
        } // <!-- liorf_yjz_lucky_boy -->
        else if (sensor == SensorType::ROBOSENSE) {
            pcl::PointCloud<RobosensePointXYZIRT>::Ptr tmpRobosenseCloudIn(new pcl::PointCloud<RobosensePointXYZIRT>());
            // Convert to robosense format
            pcl::moveFromROSMsg(currentCloudMsg, *tmpRobosenseCloudIn);
            laserCloudIn->points.resize(tmpRobosenseCloudIn->size());
            laserCloudIn->is_dense = tmpRobosenseCloudIn->is_dense;

            double start_stamptime = tmpRobosenseCloudIn->points[0].timestamp;
            for (size_t i = 0; i < tmpRobosenseCloudIn->size(); i++) {
                auto &src = tmpRobosenseCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                dst.time = src.timestamp - start_stamptime;
            }
        }
        else {
            RCLCPP_ERROR_STREAM(get_logger(), "Unknown sensor type: " << int(sensor));
            rclcpp::shutdown();
        }

        if (laserCloudIn->empty())
        {
            RCLCPP_WARN(get_logger(), "Skipping an empty point cloud");
            return false;
        }

        // Organized Ouster clouds retain missing returns as NaNs and mark the
        // message non-dense. Compact only those invalid returns; rejecting the
        // complete scan discards otherwise valid measurements.
        if (!laserCloudIn->is_dense)
        {
            const size_t inputPointCount = laserCloudIn->size();
            const auto validEnd = std::remove_if(
                laserCloudIn->points.begin(), laserCloudIn->points.end(),
                [](const PointXYZIRT &point) {
                    return !std::isfinite(point.x) ||
                           !std::isfinite(point.y) ||
                           !std::isfinite(point.z);
                });
            laserCloudIn->points.erase(validEnd, laserCloudIn->points.end());
            laserCloudIn->width = static_cast<uint32_t>(laserCloudIn->size());
            laserCloudIn->height = 1;
            laserCloudIn->is_dense = true;

            if (laserCloudIn->empty())
            {
                RCLCPP_WARN(get_logger(),
                    "Skipping a point cloud containing no finite XYZ returns");
                return false;
            }

            RCLCPP_WARN_ONCE(get_logger(),
                "Compacting non-dense point clouds (first scan retained %zu of %zu points)",
                laserCloudIn->size(), inputPointCount);
        }

        // get timestamp
        cloudHeader = currentCloudMsg.header;
        timeScanCur = stamp2Sec(cloudHeader.stamp);
        float scanDuration = 0.0f;
        for (const auto &point : laserCloudIn->points)
        {
            if (std::isfinite(point.time))
                scanDuration = std::max(scanDuration, point.time);
        }
        timeScanEnd = timeScanCur + scanDuration;

        // check ring channel
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

        // check point time
        if (deskewFlag == 0)
        {
            deskewFlag = -1;
            for (auto &field : currentCloudMsg.fields)
            {
                if (field.name == "time" || field.name == "t")
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

        // make sure IMU data available for the scan
        if (imuQueue.empty() || stamp2Sec(imuQueue.front().header.stamp) > timeScanCur || stamp2Sec(imuQueue.back().header.stamp) < timeScanEnd)
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
            if (stamp2Sec(imuQueue.front().header.stamp) < timeScanCur - 0.01)
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
            double currentImuTime = stamp2Sec(thisImuMsg.header.stamp);

            if (imuType) {
                // get roll, pitch, and yaw estimation for this scan
                if (currentImuTime <= timeScanCur)
                    imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imu_roll_init, &cloudInfo.imu_pitch_init, &cloudInfo.imu_yaw_init);
            }

            if (currentImuTime > timeScanEnd + 0.01)
                break;

            if (imuPointerCur == 0){
                imuRotX[0] = 0;
                imuRotY[0] = 0;
                imuRotZ[0] = 0;
                imuTime[0] = currentImuTime;
                ++imuPointerCur;
                continue;
            }

            // get angular velocity
            double angular_x, angular_y, angular_z;
            imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

            // integrate rotation
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
        static float sync_diff_time = (imuRate >= 300) ? 0.01 : 0.20;
        while (!odomQueue.empty())
        {
            if (stamp2Sec(odomQueue.front().header.stamp) < timeScanCur - sync_diff_time)
                odomQueue.pop_front();
            else
                break;
        }

        if (odomQueue.empty())
            return;

        if (stamp2Sec(odomQueue.front().header.stamp) > timeScanCur)
            return;

        // get start odometry at the beinning of the scan
        nav_msgs::msg::Odometry startOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
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

        // Initial guess used in mapOptimization
        cloudInfo.initial_guess_x = startOdomMsg.pose.pose.position.x;
        cloudInfo.initial_guess_y = startOdomMsg.pose.pose.position.y;
        cloudInfo.initial_guess_z = startOdomMsg.pose.pose.position.z;
        cloudInfo.initial_guess_roll  = roll;
        cloudInfo.initial_guess_pitch = pitch;
        cloudInfo.initial_guess_yaw   = yaw;

        cloudInfo.odom_available = true;

        // get end odometry at the end of the scan
        odomDeskewFlag = false;

        if (stamp2Sec(odomQueue.back().header.stamp) < timeScanEnd)
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
        } else {
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
        (void)relTime;
        *posXCur = 0; *posYCur = 0; *posZCur = 0;

        // If the sensor moves relatively slow, like walking speed, positional deskew seems to have little benefits. Thus code below is commented.

        // if (cloudInfo.odom_available == false || odomDeskewFlag == false)
        //     return;

        // float ratio = relTime / (timeScanEnd - timeScanCur);

        // *posXCur = ratio * odomIncreX;
        // *posYCur = ratio * odomIncreY;
        // *posZCur = ratio * odomIncreZ;
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
            // Organized Ouster clouds are row-major by ring, not ordered by
            // relative point time. Always deskew into the scan-header frame
            // rather than whichever point happens to be stored first.
            float rotXStart, rotYStart, rotZStart;
            findRotation(timeScanCur, &rotXStart, &rotYStart, &rotZStart);
            float posXStart, posYStart, posZStart;
            findPosition(0.0, &posXStart, &posYStart, &posZStart);
            transStartInverse = pcl::getTransformation(
                posXStart, posYStart, posZStart,
                rotXStart, rotYStart, rotZStart).inverse();
            firstPointFlag = false;
        }

        // transform points to start
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
        // range image projection
        for (int i = 0; i < cloudSize; ++i)
        {
            PointType thisPoint;
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            thisPoint.intensity = laserCloudIn->points[i].intensity;

            float range = common_lib_->pointDistance(thisPoint);
            if (range < lidarMinRange || range > lidarMaxRange)
                continue;

            int rowIdn = laserCloudIn->points[i].ring;
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            if (rowIdn % downsampleRate != 0)
                continue;

            if (i % point_filter_num != 0)
                continue;

            thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);

            fullCloud->push_back(thisPoint);
        }
    }

    void publishClouds()
    {
        cloudInfo.header = cloudHeader;
        cloudInfo.cloud_deskewed  = publishCloud(pubExtractedCloud, fullCloud, cloudHeader.stamp, lidarFrameId);
        pubLaserCloudInfo->publish(cloudInfo);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    common_lib_ = std::make_shared<CommonLib::common_lib>("mapping");

    rclcpp::NodeOptions options;
    auto IP = std::make_shared<ImageProjection>(options);

    RCLCPP_INFO(IP->get_logger(), "\033[1;32m----> Image Projection Started.\033[0m");

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
    executor.add_node(IP);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
