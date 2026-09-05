#include "liorf/msg/alignment_state.hpp"
#include "loop_constraint_utils.hpp"
#include "utility.h"

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

class TransformFusion : public ParamServer
{
public:
    std::mutex mtx;
    // Optional inter-robot alignment. This is a fleet-map -> local-map edge,
    // never the local map -> odom correction.
    std::string _fusion_topic;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subFusionTrans;
    rclcpp::Subscription<liorf::msg::AlignmentState>::SharedPtr subAlignmentState;
    std::pair<std::uint64_t, std::uint64_t> alignmentVersion{0, 0};

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subImuOdometry;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLaserOdometryGlobal;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLaserOdometryIncremental;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubImuOdometry;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubImuPath;

    rclcpp::CallbackGroup::SharedPtr callbackGroup;

    Eigen::Affine3f lidarOdomAffine;
    Eigen::Affine3f mapToOdomAffine = Eigen::Affine3f::Identity();
    Eigen::Affine3f fleetToMapAffine = Eigen::Affine3f::Identity();
    bool mapToOdomAvailable = false;
    bool fleetToMapAvailable = false;
    deque<nav_msgs::msg::Odometry> mappingGlobalQueue;
    deque<nav_msgs::msg::Odometry> mappingIncrementalQueue;

    std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> staticTfBroadcaster;
    tf2::Transform lidar2Baselink;

    double lidarOdomTime = -1;
    deque<nav_msgs::msg::Odometry> imuOdomQueue;

    explicit TransformFusion(const rclcpp::NodeOptions & options)
    : ParamServer("liorf_transformFusion", options)
    {
        tfBuffer = std::make_shared<tf2_ros::Buffer>(get_clock());
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
        tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        staticTfBroadcaster =
            std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);

        lidar2Baselink.setIdentity();
        if(lidarFrameId != baselinkFrameId)
        {
            try
            {
                auto tf = tfBuffer->lookupTransform(
                    lidarFrameId, baselinkFrameId,
                    tf2::TimePointZero, tf2::durationFromSec(3.0));
                tf2::fromMsg(tf.transform, lidar2Baselink);
            }
            catch (const tf2::TransformException & ex)
            {
                RCLCPP_ERROR(get_logger(), "%s", ex.what());
            }
        }
        _fusion_topic = declare_and_get<std::string>("mapfusion.interRobot.solid_topic", "solid");

        publishConfiguredGlobalAnchor();

        callbackGroup = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto subOpt = rclcpp::SubscriptionOptions();
        subOpt.callback_group = callbackGroup;

        if (!mapFusionFrameId.empty() && !mapFusionAnchor)
        {
            subAlignmentState = create_subscription<liorf::msg::AlignmentState>(
                prefixTopic(robot_id, _fusion_topic + "/alignment_state"), rclcpp::QoS(100),
                [this](const liorf::msg::AlignmentState::ConstSharedPtr& state) {
                    const auto version = std::make_pair(state->authority_epoch, state->revision);
                    if (state->authority_id != robot_id || state->authority_epoch == 0 ||
                        state->revision == 0 || version <= alignmentVersion) return;
                    if (state->valid && !liorf::loop_constraint::validPoseMessage(state->alignment.pose.pose)) return;
                    if (state->valid) {
                        applyFusionTransform(std::make_shared<nav_msgs::msg::Odometry>(state->alignment));
                    } else {
                        std::lock_guard<std::mutex> lock(mtx);
                        fleetToMapAvailable = false;
                    }
                    alignmentVersion = version;
                }, subOpt);
            subFusionTrans = create_subscription<nav_msgs::msg::Odometry>(
                prefixTopic(robot_id, _fusion_topic + "/trans_map"), rclcpp::QoS(20),
                std::bind(&TransformFusion::FusionTransHandler, this,
                    std::placeholders::_1), subOpt);
        }

        subLaserOdometryGlobal = create_subscription<nav_msgs::msg::Odometry>(
            prefixTopic(robot_id, "liorf/mapping/odometry"), rclcpp::QoS(5),
            std::bind(&TransformFusion::mappingGlobalHandler, this,
                std::placeholders::_1), subOpt);
        subLaserOdometryIncremental =
            create_subscription<nav_msgs::msg::Odometry>(
                prefixTopic(robot_id, "liorf/mapping/odometry_incremental"),
                rclcpp::QoS(5),
                std::bind(&TransformFusion::mappingIncrementalHandler, this,
                    std::placeholders::_1), subOpt);
        subImuOdometry   = create_subscription<nav_msgs::msg::Odometry>(
            prefixTopic(robot_id, odomTopic + "_incremental"), rclcpp::QoS(2000),
            std::bind(&TransformFusion::imuOdometryHandler, this, std::placeholders::_1), subOpt);

        pubImuOdometry   = create_publisher<nav_msgs::msg::Odometry>(prefixTopic(robot_id, odomTopic), 2000);
        pubImuPath       = create_publisher<nav_msgs::msg::Path>    (prefixTopic(robot_id, "liorf/imu/path"), 1);
    }

    Eigen::Affine3f odom2affine(const nav_msgs::msg::Odometry & odom)
    {
        double x, y, z, roll, pitch, yaw;
        x = odom.pose.pose.position.x;
        y = odom.pose.pose.position.y;
        z = odom.pose.pose.position.z;
        tf2::Quaternion orientation;
        tf2::fromMsg(odom.pose.pose.orientation, orientation);
        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        return pcl::getTransformation(x, y, z, roll, pitch, yaw);
    }

    void publishConfiguredGlobalAnchor()
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = now();

        if (geographicFrameMode ==
            liorf::frames::GeographicFrameMode::ECEF_ANCHORED)
        {
            const liorf::frames::GeographicMapAnchor anchor(mapDatum);
            const Eigen::Isometry3d earth_from_map = anchor.earthFromMap();
            const Eigen::Quaterniond q(earth_from_map.rotation());
            transform.header.frame_id = earthFrameId;
            transform.child_frame_id = mapFrameId;
            transform.transform.translation.x = earth_from_map.translation().x();
            transform.transform.translation.y = earth_from_map.translation().y();
            transform.transform.translation.z = earth_from_map.translation().z();
            transform.transform.rotation.x = q.x();
            transform.transform.rotation.y = q.y();
            transform.transform.rotation.z = q.z();
            transform.transform.rotation.w = q.w();
            staticTfBroadcaster->sendTransform(transform);
            RCLCPP_INFO(
                get_logger(),
                "Publishing WGS-84 ECEF anchor %s -> %s at "
                "(%.9f deg, %.9f deg, %.3f m ellipsoid height).",
                earthFrameId.c_str(), mapFrameId.c_str(),
                mapDatum.latitude_deg, mapDatum.longitude_deg,
                mapDatum.ellipsoid_height_m);
        }
        else if (mapFusionAnchor && !mapFusionFrameId.empty())
        {
            transform.header.frame_id = mapFusionFrameId;
            transform.child_frame_id = mapFrameId;
            transform.transform.rotation.w = 1.0;
            staticTfBroadcaster->sendTransform(transform);
            RCLCPP_INFO(
                get_logger(), "Defining map-fusion gauge %s -> %s as identity.",
                mapFusionFrameId.c_str(), mapFrameId.c_str());
        }
    }

    void sendMapToOdomTransform(const builtin_interfaces::msg::Time & stamp)
    {
        if (!mapToOdomAvailable)
            return;

        const Eigen::Quaternionf q(mapToOdomAffine.rotation());
        geometry_msgs::msg::TransformStamped map_to_odom;
        map_to_odom.header.stamp = stamp;
        map_to_odom.header.frame_id = mapFrameId;
        map_to_odom.child_frame_id = odometryFrameId;
        map_to_odom.transform.translation.x = mapToOdomAffine.translation().x();
        map_to_odom.transform.translation.y = mapToOdomAffine.translation().y();
        map_to_odom.transform.translation.z = mapToOdomAffine.translation().z();
        map_to_odom.transform.rotation.x = q.x();
        map_to_odom.transform.rotation.y = q.y();
        map_to_odom.transform.rotation.z = q.z();
        map_to_odom.transform.rotation.w = q.w();
        tfBroadcaster->sendTransform(map_to_odom);
    }

    void sendFleetToMapTransform(const builtin_interfaces::msg::Time & stamp)
    {
        if (!fleetToMapAvailable || mapFusionFrameId.empty())
            return;

        const Eigen::Quaternionf q(fleetToMapAffine.rotation());
        geometry_msgs::msg::TransformStamped fleet_to_map;
        fleet_to_map.header.stamp = stamp;
        fleet_to_map.header.frame_id = mapFusionFrameId;
        fleet_to_map.child_frame_id = mapFrameId;
        fleet_to_map.transform.translation.x = fleetToMapAffine.translation().x();
        fleet_to_map.transform.translation.y = fleetToMapAffine.translation().y();
        fleet_to_map.transform.translation.z = fleetToMapAffine.translation().z();
        fleet_to_map.transform.rotation.x = q.x();
        fleet_to_map.transform.rotation.y = q.y();
        fleet_to_map.transform.rotation.z = q.z();
        fleet_to_map.transform.rotation.w = q.w();
        tfBroadcaster->sendTransform(fleet_to_map);
    }

    void FusionTransHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg)
    {
        if (alignmentVersion.first != 0) return;
        applyFusionTransform(odomMsg);
    }

    void applyFusionTransform(const nav_msgs::msg::Odometry::SharedPtr odomMsg)
    {
        if (!liorf::loop_constraint::validPoseMessage(odomMsg->pose.pose)) return;
        const std::string parent =
            liorf::frames::normalizeFrameId(odomMsg->header.frame_id);
        const std::string child =
            liorf::frames::normalizeFrameId(odomMsg->child_frame_id);
        if (parent != mapFusionFrameId || child != mapFrameId)
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Rejecting map-fusion transform with contract %s -> %s; "
                "expected %s -> %s.",
                parent.c_str(), child.c_str(), mapFusionFrameId.c_str(),
                mapFrameId.c_str());
            return;
        }

        std::lock_guard<std::mutex> lock(mtx);
        fleetToMapAffine = odom2affine(*odomMsg);
        fleetToMapAvailable = true;
        sendFleetToMapTransform(odomMsg->header.stamp);
    }

    void updateMapToOdomCorrection()
    {
        constexpr double stamp_tolerance = 1e-6;
        while (!mappingGlobalQueue.empty() &&
               !mappingIncrementalQueue.empty())
        {
            const double global_time =
                stamp2Sec(mappingGlobalQueue.front().header.stamp);
            const double incremental_time =
                stamp2Sec(mappingIncrementalQueue.front().header.stamp);
            const double difference = global_time - incremental_time;
            if (std::abs(difference) <= stamp_tolerance)
            {
                const Eigen::Affine3f map_from_lidar =
                    odom2affine(mappingGlobalQueue.front());
                const Eigen::Affine3f odom_from_lidar =
                    odom2affine(mappingIncrementalQueue.front());
                mapToOdomAffine =
                    map_from_lidar * odom_from_lidar.inverse();
                mapToOdomAvailable = true;
                const auto stamp = mappingGlobalQueue.front().header.stamp;
                mappingGlobalQueue.pop_front();
                mappingIncrementalQueue.pop_front();
                sendMapToOdomTransform(stamp);
            }
            else if (difference < 0.0)
            {
                mappingGlobalQueue.pop_front();
            }
            else
            {
                mappingIncrementalQueue.pop_front();
            }
        }
    }

    void mappingGlobalHandler(
        const nav_msgs::msg::Odometry::SharedPtr odomMsg)
    {
        if (liorf::frames::normalizeFrameId(odomMsg->header.frame_id) !=
            mapFrameId)
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Rejecting corrected mapping odometry in frame '%s'; expected '%s'.",
                odomMsg->header.frame_id.c_str(), mapFrameId.c_str());
            return;
        }

        std::lock_guard<std::mutex> lock(mtx);
        mappingGlobalQueue.push_back(*odomMsg);
        if (mappingGlobalQueue.size() > 50)
            mappingGlobalQueue.pop_front();
        updateMapToOdomCorrection();
    }

    void mappingIncrementalHandler(
        const nav_msgs::msg::Odometry::SharedPtr odomMsg)
    {
        if (liorf::frames::normalizeFrameId(odomMsg->header.frame_id) !=
            odometryFrameId)
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Rejecting incremental mapping odometry in frame '%s'; expected '%s'.",
                odomMsg->header.frame_id.c_str(), odometryFrameId.c_str());
            return;
        }

        std::lock_guard<std::mutex> lock(mtx);
        lidarOdomAffine = odom2affine(*odomMsg);
        lidarOdomTime = stamp2Sec(odomMsg->header.stamp);
        mappingIncrementalQueue.push_back(*odomMsg);
        if (mappingIncrementalQueue.size() > 50)
            mappingIncrementalQueue.pop_front();
        updateMapToOdomCorrection();
    }

    void imuOdometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);

        // Dynamic corrections are repeated at the high-rate odometry stamp so
        // consumers can query a coherent live chain.
        sendMapToOdomTransform(odomMsg->header.stamp);
        sendFleetToMapTransform(odomMsg->header.stamp);

        imuOdomQueue.push_back(*odomMsg);

        // get latest odometry (at current IMU stamp)
        if (lidarOdomTime == -1)
            return;
        while (!imuOdomQueue.empty())
        {
            if (stamp2Sec(imuOdomQueue.front().header.stamp) <= lidarOdomTime)
                imuOdomQueue.pop_front();
            else
                break;
        }
        if (imuOdomQueue.empty())
            return;
        Eigen::Affine3f imuOdomAffineFront = odom2affine(imuOdomQueue.front());
        Eigen::Affine3f imuOdomAffineBack = odom2affine(imuOdomQueue.back());
        Eigen::Affine3f imuOdomAffineIncre = imuOdomAffineFront.inverse() * imuOdomAffineBack;
        Eigen::Affine3f imuOdomAffineLast = lidarOdomAffine * imuOdomAffineIncre;
        // The pre-integrated state is a LiDAR pose. Convert it to the output
        // body pose before publishing both Odometry and TF.
        tf2::Transform odom_to_body;
        const Eigen::Quaternionf lidar_rotation(imuOdomAffineLast.rotation());
        odom_to_body.setOrigin(tf2::Vector3(
            imuOdomAffineLast.translation().x(),
            imuOdomAffineLast.translation().y(),
            imuOdomAffineLast.translation().z()));
        odom_to_body.setRotation(tf2::Quaternion(
            lidar_rotation.x(), lidar_rotation.y(), lidar_rotation.z(),
            lidar_rotation.w()));
        if (lidarFrameId != baselinkFrameId)
            odom_to_body = odom_to_body * lidar2Baselink;

        nav_msgs::msg::Odometry laserOdometry = imuOdomQueue.back();
        laserOdometry.header.frame_id = odometryFrameId;
        laserOdometry.child_frame_id = baselinkFrameId;
        laserOdometry.pose.pose.position.x = odom_to_body.getOrigin().x();
        laserOdometry.pose.pose.position.y = odom_to_body.getOrigin().y();
        laserOdometry.pose.pose.position.z = odom_to_body.getOrigin().z();
        laserOdometry.pose.pose.orientation =
            tf2::toMsg(odom_to_body.getRotation());
        pubImuOdometry->publish(laserOdometry);

        // publish tf
        geometry_msgs::msg::TransformStamped odom_2_baselink;
        odom_2_baselink.header.stamp = odomMsg->header.stamp;
        odom_2_baselink.header.frame_id = odometryFrameId;
        odom_2_baselink.child_frame_id = baselinkFrameId;
        odom_2_baselink.transform = tf2::toMsg(odom_to_body);
        tfBroadcaster->sendTransform(odom_2_baselink);

        // publish IMU path
        static nav_msgs::msg::Path imuPath;
        static double last_path_time = -1;
        double imuTime = stamp2Sec(imuOdomQueue.back().header.stamp);
        if (imuTime - last_path_time > 0.1)
        {
            last_path_time = imuTime;
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.stamp = imuOdomQueue.back().header.stamp;
            pose_stamped.header.frame_id = odometryFrameId;
            pose_stamped.pose = laserOdometry.pose.pose;
            imuPath.poses.push_back(pose_stamped);
            while(!imuPath.poses.empty() && stamp2Sec(imuPath.poses.front().header.stamp) < lidarOdomTime - 0.1) //1.0 -> 0.1
                imuPath.poses.erase(imuPath.poses.begin());
            if (pubImuPath->get_subscription_count() != 0)
            {
                imuPath.header.stamp = imuOdomQueue.back().header.stamp;
                imuPath.header.frame_id = odometryFrameId;
                pubImuPath->publish(imuPath);
            }
        }
    }
};

class IMUPreintegration : public ParamServer
{
public:

    std::mutex mtx;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdometry;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubImuOdometry;

    // Mutually exclusive groups keep each stream in order (the IMU queues and
    // gtsam pre-integration require monotonic timestamps) while still letting
    // IMU intake and the odometry optimisation run on separate threads, as the
    // ROS 1 MultiThreadedSpinner did.
    rclcpp::CallbackGroup::SharedPtr imuCallbackGroup;
    rclcpp::CallbackGroup::SharedPtr odomCallbackGroup;

    bool systemInitialized = false;

    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;
    gtsam::Vector noiseModelBetweenBias;


    gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
    gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;

    std::deque<sensor_msgs::msg::Imu> imuQueOpt;
    std::deque<sensor_msgs::msg::Imu> imuQueImu;

    gtsam::Pose3 prevPose_;
    gtsam::Vector3 prevVel_;
    gtsam::NavState prevState_;
    gtsam::imuBias::ConstantBias prevBias_;

    gtsam::NavState prevStateOdom;
    gtsam::imuBias::ConstantBias prevBiasOdom;

    bool doneFirstOpt = false;
    double lastImuT_received = -1;
    double lastImuT_imu = -1;
    double lastImuT_opt = -1;
    uint64_t rejectedImuTimestampCount = 0;

    gtsam::ISAM2 optimizer;
    gtsam::NonlinearFactorGraph graphFactors;
    gtsam::Values graphValues;

    const double delta_t = 0;

    int key = 1;

    // T_bl: tramsform points from lidar frame to imu frame
    gtsam::Pose3 imu2Lidar;
    // T_lb: tramsform points from imu frame to lidar frame
    gtsam::Pose3 lidar2Imu;

    explicit IMUPreintegration(const rclcpp::NodeOptions & options)
    : ParamServer("liorf_imuPreintegration", options)
    {
        imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(-extTrans.x(), -extTrans.y(), -extTrans.z()));
        lidar2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0), gtsam::Point3(extTrans.x(), extTrans.y(), extTrans.z()));

        imuCallbackGroup  = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        odomCallbackGroup = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto imuOpt = rclcpp::SubscriptionOptions();
        imuOpt.callback_group = imuCallbackGroup;
        auto odomOpt = rclcpp::SubscriptionOptions();
        odomOpt.callback_group = odomCallbackGroup;

        subImu      = create_subscription<sensor_msgs::msg::Imu>(
            prefixTopic(robot_id, imuTopic), rclcpp::SensorDataQoS().keep_last(2000),
            std::bind(&IMUPreintegration::imuHandler, this, std::placeholders::_1), imuOpt);
        subOdometry = create_subscription<nav_msgs::msg::Odometry>(
            prefixTopic(robot_id, "liorf/mapping/odometry_incremental"), rclcpp::QoS(5),
            std::bind(&IMUPreintegration::odometryHandler, this, std::placeholders::_1), odomOpt);

        pubImuOdometry = create_publisher<nav_msgs::msg::Odometry>(prefixTopic(robot_id, odomTopic + "_incremental"), 2000);

        auto p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
        p->accelerometerCovariance  = gtsam::Matrix33::Identity(3,3) * pow(imuAccNoise, 2); // acc white noise in continuous
        p->gyroscopeCovariance      = gtsam::Matrix33::Identity(3,3) * pow(imuGyrNoise, 2); // gyro white noise in continuous
        p->integrationCovariance    = gtsam::Matrix33::Identity(3,3) * pow(1e-4, 2); // error committed in integrating position from velocities
        gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());; // assume zero initial bias

        priorPoseNoise  = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
        priorVelNoise   = gtsam::noiseModel::Isotropic::Sigma(3, 1e4); // m/s
        priorBiasNoise  = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3); // 1e-2 ~ 1e-3 seems to be good
        correctionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished()); // rad,rad,rad,m, m, m
        correctionNoise2 = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished()); // rad,rad,rad,m, m, m
        noiseModelBetweenBias = (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();

        imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for IMU message thread
        imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for optimization
    }

    ~IMUPreintegration()
    {
        delete imuIntegratorImu_;
        delete imuIntegratorOpt_;
    }

    void resetOptimization()
    {
        gtsam::ISAM2Params optParameters;
        optParameters.relinearizeThreshold = 0.1;
        optParameters.relinearizeSkip = 1;
        optimizer = gtsam::ISAM2(optParameters);

        gtsam::NonlinearFactorGraph newGraphFactors;
        graphFactors = newGraphFactors;

        gtsam::Values NewGraphValues;
        graphValues = NewGraphValues;
    }

    void resetParams()
    {
        lastImuT_imu = -1;
        doneFirstOpt = false;
        systemInitialized = false;
    }

    bool validIntegrationStep(double dt, const char * path)
    {
        if (std::isfinite(dt) && dt > 0.0)
            return true;

        ++rejectedImuTimestampCount;
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Dropping a non-positive/non-finite IMU integration step in %s "
            "(dt=%.9g, rejected=%lu).",
            path, dt, static_cast<unsigned long>(rejectedImuTimestampCount));
        return false;
    }

    void odometryHandler(const nav_msgs::msg::Odometry::SharedPtr odomMsg)
    {
        std::lock_guard<std::mutex> lock(mtx);

        double currentCorrectionTime = ROS_TIME(odomMsg);

        // make sure we have imu data to integrate
        if (imuQueOpt.empty())
            return;

        float p_x = odomMsg->pose.pose.position.x;
        float p_y = odomMsg->pose.pose.position.y;
        float p_z = odomMsg->pose.pose.position.z;
        float r_x = odomMsg->pose.pose.orientation.x;
        float r_y = odomMsg->pose.pose.orientation.y;
        float r_z = odomMsg->pose.pose.orientation.z;
        float r_w = odomMsg->pose.pose.orientation.w;
        bool degenerate = (int)odomMsg->pose.covariance[0] == 1 ? true : false;
        gtsam::Pose3 lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));


        // 0. initialize system
        if (systemInitialized == false)
        {
            resetOptimization();

            // pop old IMU message
            while (!imuQueOpt.empty())
            {
                if (ROS_TIME(&imuQueOpt.front()) < currentCorrectionTime - delta_t)
                {
                    lastImuT_opt = ROS_TIME(&imuQueOpt.front());
                    imuQueOpt.pop_front();
                }
                else
                    break;
            }
            // initial pose
            prevPose_ = lidarPose.compose(lidar2Imu);
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
            graphFactors.add(priorPose);
            // initial velocity
            prevVel_ = gtsam::Vector3(0, 0, 0);
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
            graphFactors.add(priorVel);
            // initial bias
            prevBias_ = gtsam::imuBias::ConstantBias();
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
            graphFactors.add(priorBias);
            // add values
            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);
            // optimize once
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();

            imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
            imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

            key = 1;
            systemInitialized = true;
            return;
        }


        // reset graph for speed
        if (key == 100)
        {
            // get updated noise before reset
            gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(X(key-1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise  = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(V(key-1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(optimizer.marginalCovariance(B(key-1)));
            // reset graph
            resetOptimization();
            // add pose
            gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, updatedPoseNoise);
            graphFactors.add(priorPose);
            // add velocity
            gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), prevVel_, updatedVelNoise);
            graphFactors.add(priorVel);
            // add bias
            gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, updatedBiasNoise);
            graphFactors.add(priorBias);
            // add values
            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);
            // optimize once
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();

            key = 1;
        }


        // 1. integrate imu data and optimize
        while (!imuQueOpt.empty())
        {
            // pop and integrate imu data that is between two optimizations
            sensor_msgs::msg::Imu *thisImu = &imuQueOpt.front();
            double imuTime = ROS_TIME(thisImu);
            if (imuTime < currentCorrectionTime - delta_t)
            {
                double dt = (lastImuT_opt < 0) ? (1.0 / imuRate) : (imuTime - lastImuT_opt);
                if (!validIntegrationStep(dt, "optimization"))
                {
                    imuQueOpt.pop_front();
                    continue;
                }
                imuIntegratorOpt_->integrateMeasurement(
                        gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);

                lastImuT_opt = imuTime;
                imuQueOpt.pop_front();
            }
            else
                break;
        }
        // add imu factor to graph
        const gtsam::PreintegratedImuMeasurements& preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*imuIntegratorOpt_);
        gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
        graphFactors.add(imu_factor);
        // add imu bias between factor
        graphFactors.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                         gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegratorOpt_->deltaTij()) * noiseModelBetweenBias)));
        // add pose factor
        gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu);
        gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), curPose, degenerate ? correctionNoise2 : correctionNoise);
        graphFactors.add(pose_factor);
        // insert predicted values
        gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
        graphValues.insert(X(key), propState_.pose());
        graphValues.insert(V(key), propState_.v());
        graphValues.insert(B(key), prevBias_);
        // optimize
        optimizer.update(graphFactors, graphValues);
        optimizer.update();
        graphFactors.resize(0);
        graphValues.clear();
        // Overwrite the beginning of the preintegration for the next step.
        gtsam::Values result = optimizer.calculateEstimate();
        prevPose_  = result.at<gtsam::Pose3>(X(key));
        prevVel_   = result.at<gtsam::Vector3>(V(key));
        prevState_ = gtsam::NavState(prevPose_, prevVel_);
        prevBias_  = result.at<gtsam::imuBias::ConstantBias>(B(key));
        // Reset the optimization preintegration object.
        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);
        // check optimization
        if (failureDetection(prevVel_, prevBias_))
        {
            resetParams();
            return;
        }


        // 2. after optiization, re-propagate imu odometry preintegration
        prevStateOdom = prevState_;
        prevBiasOdom  = prevBias_;
        // first pop imu message older than current correction data
        double lastImuQT = -1;
        while (!imuQueImu.empty() && ROS_TIME(&imuQueImu.front()) < currentCorrectionTime - delta_t)
        {
            lastImuQT = ROS_TIME(&imuQueImu.front());
            imuQueImu.pop_front();
        }
        // repropogate
        if (!imuQueImu.empty())
        {
            // reset bias use the newly optimized bias
            imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
            // integrate imu message from the beginning of this optimization
            for (int i = 0; i < (int)imuQueImu.size(); ++i)
            {
                sensor_msgs::msg::Imu *thisImu = &imuQueImu[i];
                double imuTime = ROS_TIME(thisImu);
                double dt = (lastImuQT < 0) ? (1.0 / imuRate) :(imuTime - lastImuQT);

                if (!validIntegrationStep(dt, "repropagation"))
                    continue;

                imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y, thisImu->linear_acceleration.z),
                                                        gtsam::Vector3(thisImu->angular_velocity.x,    thisImu->angular_velocity.y,    thisImu->angular_velocity.z), dt);
                lastImuQT = imuTime;
            }
        }

        ++key;
        doneFirstOpt = true;
    }

    bool failureDetection(const gtsam::Vector3& velCur, const gtsam::imuBias::ConstantBias& biasCur)
    {
        Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
        if (vel.norm() > 30)
        {
            RCLCPP_WARN(get_logger(), "Large velocity, reset IMU-preintegration!");
            return true;
        }

        Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());
        Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());
        if (ba.norm() > 1.0 || bg.norm() > 1.0)
        {
            RCLCPP_WARN(get_logger(), "Large bias, reset IMU-preintegration!");
            return true;
        }

        return false;
    }

    void imuHandler(const sensor_msgs::msg::Imu::SharedPtr imu_raw)
    {
        std::lock_guard<std::mutex> lock(mtx);

        sensor_msgs::msg::Imu thisImu = imuConverter(*imu_raw);

        const double imuTime = ROS_TIME(&thisImu);
        if (!std::isfinite(imuTime) ||
            (lastImuT_received >= 0.0 && imuTime <= lastImuT_received))
        {
            ++rejectedImuTimestampCount;
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Dropping out-of-order IMU sample (stamp=%.9f, previous=%.9f, "
                "rejected=%lu).",
                imuTime, lastImuT_received,
                static_cast<unsigned long>(rejectedImuTimestampCount));
            return;
        }
        lastImuT_received = imuTime;

        imuQueOpt.push_back(thisImu);
        imuQueImu.push_back(thisImu);

        if (doneFirstOpt == false)
            return;

        double dt = (lastImuT_imu < 0) ? (1.0 / imuRate) : (imuTime - lastImuT_imu);
        if (!validIntegrationStep(dt, "real-time propagation"))
            return;
        lastImuT_imu = imuTime;

        // integrate this single imu message
        imuIntegratorImu_->integrateMeasurement(gtsam::Vector3(thisImu.linear_acceleration.x, thisImu.linear_acceleration.y, thisImu.linear_acceleration.z),
                                                gtsam::Vector3(thisImu.angular_velocity.x,    thisImu.angular_velocity.y,    thisImu.angular_velocity.z), dt);

        // predict odometry
        gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);

        // publish odometry
        nav_msgs::msg::Odometry odometry;
        odometry.header.stamp = thisImu.header.stamp;
        odometry.header.frame_id = odometryFrameId;
        odometry.child_frame_id = lidarFrameId;

        // transform imu pose to ldiar
        gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
        gtsam::Pose3 lidarPose = imuPose.compose(imu2Lidar);

        odometry.pose.pose.position.x = lidarPose.translation().x();
        odometry.pose.pose.position.y = lidarPose.translation().y();
        odometry.pose.pose.position.z = lidarPose.translation().z();
        odometry.pose.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
        odometry.pose.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
        odometry.pose.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
        odometry.pose.pose.orientation.w = lidarPose.rotation().toQuaternion().w();

        odometry.twist.twist.linear.x = currentState.velocity().x();
        odometry.twist.twist.linear.y = currentState.velocity().y();
        odometry.twist.twist.linear.z = currentState.velocity().z();
        odometry.twist.twist.angular.x = thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
        odometry.twist.twist.angular.y = thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
        odometry.twist.twist.angular.z = thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();
        pubImuOdometry->publish(odometry);
    }
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;

    auto ImuP = std::make_shared<IMUPreintegration>(options);
    auto TF = std::make_shared<TransformFusion>(options);

    RCLCPP_INFO(ImuP->get_logger(), "\033[1;32m----> IMU Preintegration Started.\033[0m");

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(ImuP);
    executor.add_node(TF);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
