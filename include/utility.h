#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_
#define PCL_NO_PRECOMPILE
// <!-- liorf_yjz_lucky_boy -->
#include <rclcpp/rclcpp.hpp>
#include "ros_compat.h"

#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <common_lib.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/opencv.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <unistd.h>

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
#include <stdexcept>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

using namespace std;

typedef pcl::PointXYZI PointType;

// <!-- liorf_localization_yjz_lucky_boy -->
inline std::shared_ptr<CommonLib::common_lib> common_lib_;

enum class SensorType { VELODYNE, OUSTER, LIVOX, ROBOSENSE, MULRAN};

class ParamServer : public rclcpp::Node
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

    // Lidar Sensor Configuration
    SensorType sensor;
    int N_SCAN;
    int Horizon_SCAN;
    int downsampleRate;
    int point_filter_num;
    float lidarMinRange;
    float lidarMaxRange;

    // IMU
    int imuType;
    float imuRate;
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

    // voxel filter paprams
    float mappingSurfLeafSize ;
    float surroundingKeyframeMapLeafSize;
    float loopClosureICPSurfLeafSize ;

    // RESPLE/X-ICP-style observable-subspace scan matching. Information
    // thresholds are scale-free shares within the translation or rotation
    // point-to-plane Hessian block. A positive full threshold enables that
    // block; a zero/full pair leaves it fully admitted.
    bool observabilityAwareScanMatching;
    double minimumTranslationInformationShare;
    double fullTranslationInformationShare;
    double minimumRotationInformationShare;
    double fullRotationInformationShare;
    double partialTranslationCorrectionBudget;
    double partialRotationCorrectionBudget;

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

    int number_print;

    ParamServer(const std::string & node_name, const rclcpp::NodeOptions & options)
    : Node(node_name, options)
    {
        number_print = declare_and_get<int>("no", 100);
        robot_id     = declare_and_get<std::string>("robot_id", "jackal0");

        pointCloudTopic = declare_and_get<std::string>("liorf.pointCloudTopic", "points_raw");
        imuTopic        = declare_and_get<std::string>("liorf.imuTopic", "imu_correct");
        odomTopic       = declare_and_get<std::string>("liorf.odomTopic", "odometry/imu");
        gpsTopic        = declare_and_get<std::string>("liorf.gpsTopic", "odometry/gps");

        lidarFrame    = declare_and_get<std::string>("liorf.lidarFrame", "base_link");
        baselinkFrame = declare_and_get<std::string>("liorf.baselinkFrame", "base_link");
        odometryFrame = declare_and_get<std::string>("liorf.odometryFrame", "odom");
        mapFrame      = declare_and_get<std::string>("liorf.mapFrame", "map");

        useImuHeadingInitialization = declare_and_get<bool>("liorf.useImuHeadingInitialization", false);
        useGpsElevation             = declare_and_get<bool>("liorf.useGpsElevation", false);
        gpsCovThreshold             = declare_and_get<double>("liorf.gpsCovThreshold", 2.0);
        poseCovThreshold            = declare_and_get<double>("liorf.poseCovThreshold", 25.0);

        savePCD          = declare_and_get<bool>("liorf.savePCD", false);
        savePCDDirectory = declare_and_get<std::string>("liorf.savePCDDirectory", "/Downloads/LOAM/");

        std::string sensorStr = declare_and_get<std::string>("liorf.sensor", "");
        if (sensorStr == "velodyne")
        {
            sensor = SensorType::VELODYNE;
        }
        else if (sensorStr == "ouster")
        {
            sensor = SensorType::OUSTER;
        }
        else if (sensorStr == "livox")
        {
            sensor = SensorType::LIVOX;
        } else if  (sensorStr == "robosense") {
            sensor = SensorType::ROBOSENSE;
        }
        else if (sensorStr == "mulran")
        {
            sensor = SensorType::MULRAN;
        }
        else {
            RCLCPP_ERROR_STREAM(get_logger(),
                "Invalid sensor type (must be either 'velodyne' or 'ouster' or 'livox' or 'robosense' or 'mulran'): " << sensorStr);
            rclcpp::shutdown();
        }

        N_SCAN           = declare_and_get<int>("liorf.N_SCAN", 16);
        Horizon_SCAN     = declare_and_get<int>("liorf.Horizon_SCAN", 1800);
        downsampleRate   = declare_and_get<int>("liorf.downsampleRate", 1);
        point_filter_num = declare_and_get<int>("liorf.point_filter_num", 3);
        lidarMinRange    = declare_and_get<double>("liorf.lidarMinRange", 1.0);
        lidarMaxRange    = declare_and_get<double>("liorf.lidarMaxRange", 1000.0);

        imuType      = declare_and_get<int>("liorf.imuType", 0);
        imuRate      = declare_and_get<double>("liorf.imuRate", 500.0);
        imuAccNoise  = declare_and_get<double>("liorf.imuAccNoise", 0.01);
        imuGyrNoise  = declare_and_get<double>("liorf.imuGyrNoise", 0.001);
        imuAccBiasN  = declare_and_get<double>("liorf.imuAccBiasN", 0.0002);
        imuGyrBiasN  = declare_and_get<double>("liorf.imuGyrBiasN", 0.00003);
        imuGravity   = declare_and_get<double>("liorf.imuGravity", 9.80511);
        imuRPYWeight = declare_and_get<double>("liorf.imuRPYWeight", 0.01);

        extRotV   = declare_and_get<std::vector<double>>("liorf.extrinsicRot", std::vector<double>());
        extRPYV   = declare_and_get<std::vector<double>>("liorf.extrinsicRPY", std::vector<double>());
        extTransV = declare_and_get<std::vector<double>>("liorf.extrinsicTrans", std::vector<double>());
        if (extRotV.size() != 9 || extRPYV.size() != 9 || extTransV.size() != 3)
        {
            RCLCPP_ERROR(get_logger(),
                "extrinsicRot/extrinsicRPY must have 9 elements and extrinsicTrans 3 "
                "(got %zu/%zu/%zu); falling back to identity.",
                extRotV.size(), extRPYV.size(), extTransV.size());
            extRotV   = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
            extRPYV   = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
            extTransV = {0.0, 0.0, 0.0};
        }
        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
        extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
        extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
        extQRPY = Eigen::Quaterniond(extRPY).inverse();

        mappingSurfLeafSize           = declare_and_get<double>("liorf.mappingSurfLeafSize", 0.2);
        surroundingKeyframeMapLeafSize = declare_and_get<double>("liorf.surroundingKeyframeMapLeafSize", 0.2);

        observabilityAwareScanMatching =
            declare_and_get<bool>("liorf.observabilityAwareScanMatching", false);
        minimumTranslationInformationShare =
            declare_and_get<double>("liorf.minimumTranslationInformationShare", 0.005);
        fullTranslationInformationShare =
            declare_and_get<double>("liorf.fullTranslationInformationShare", 0.02);
        minimumRotationInformationShare =
            declare_and_get<double>("liorf.minimumRotationInformationShare", 0.005);
        fullRotationInformationShare =
            declare_and_get<double>("liorf.fullRotationInformationShare", 0.02);
        partialTranslationCorrectionBudget =
            declare_and_get<double>("liorf.partialTranslationCorrectionBudget", 0.15);
        partialRotationCorrectionBudget =
            declare_and_get<double>("liorf.partialRotationCorrectionBudget", 0.05);

        const auto valid_information_band = [](double minimum_share, double full_share) {
            return std::isfinite(minimum_share) && std::isfinite(full_share) &&
                   minimum_share >= 0.0 && full_share >= minimum_share &&
                   full_share <= 1.0;
        };
        if (!valid_information_band(minimumTranslationInformationShare,
                                    fullTranslationInformationShare) ||
            !valid_information_band(minimumRotationInformationShare,
                                    fullRotationInformationShare) ||
            !std::isfinite(partialTranslationCorrectionBudget) ||
            !std::isfinite(partialRotationCorrectionBudget) ||
            partialTranslationCorrectionBudget < 0.0 ||
            partialRotationCorrectionBudget < 0.0)
        {
            throw std::invalid_argument(
                "invalid observable scan-matching information band or correction budget");
        }

        z_tollerance        = declare_and_get<double>("liorf.z_tollerance", FLT_MAX);
        rotation_tollerance = declare_and_get<double>("liorf.rotation_tollerance", FLT_MAX);

        numberOfCores          = declare_and_get<int>("liorf.numberOfCores", 2);
        mappingProcessInterval = declare_and_get<double>("liorf.mappingProcessInterval", 0.15);

        surroundingkeyframeAddingDistThreshold  = declare_and_get<double>("liorf.surroundingkeyframeAddingDistThreshold", 1.0);
        surroundingkeyframeAddingAngleThreshold = declare_and_get<double>("liorf.surroundingkeyframeAddingAngleThreshold", 0.2);
        surroundingKeyframeDensity              = declare_and_get<double>("liorf.surroundingKeyframeDensity", 1.0);
        loopClosureICPSurfLeafSize              = declare_and_get<double>("liorf.loopClosureICPSurfLeafSize", 0.3);
        surroundingKeyframeSearchRadius         = declare_and_get<double>("liorf.surroundingKeyframeSearchRadius", 50.0);

        loopClosureEnableFlag         = declare_and_get<bool>("liorf.loopClosureEnableFlag", false);
        loopClosureFrequency          = declare_and_get<double>("liorf.loopClosureFrequency", 1.0);
        surroundingKeyframeSize       = declare_and_get<int>("liorf.surroundingKeyframeSize", 50);
        historyKeyframeSearchRadius   = declare_and_get<double>("liorf.historyKeyframeSearchRadius", 10.0);
        historyKeyframeSearchTimeDiff = declare_and_get<double>("liorf.historyKeyframeSearchTimeDiff", 30.0);
        historyKeyframeSearchNum      = declare_and_get<int>("liorf.historyKeyframeSearchNum", 25);
        historyKeyframeFitnessScore   = declare_and_get<double>("liorf.historyKeyframeFitnessScore", 0.3);

        globalMapVisualizationSearchRadius = declare_and_get<double>("liorf.globalMapVisualizationSearchRadius", 1e3);
        globalMapVisualizationPoseDensity  = declare_and_get<double>("liorf.globalMapVisualizationPoseDensity", 10.0);
        globalMapVisualizationLeafSize     = declare_and_get<double>("liorf.globalMapVisualizationLeafSize", 1.0);

        usleep(100);
    }

    // Declares a parameter (if it is not declared yet) and returns its value.
    template<typename T>
    T declare_and_get(const std::string & name, const T & default_value)
    {
        if (!this->has_parameter(name))
            this->declare_parameter<T>(name, default_value);
        T value{};
        this->get_parameter(name, value);
        return value;
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

        if (imuType) {
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
        }

        return imu_out;
    }
};

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

#endif
