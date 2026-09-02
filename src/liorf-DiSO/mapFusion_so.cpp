//
// Created by yewei on 8/31/20.
// Modified by Hogyun Kim on 11/07/24

//

//msg
#include "liorf/msg/cloud_info.hpp"
#include "liorf/msg/context_info.hpp"
#include "liorf/msg/loop_constraint.hpp"
#include "liorf/msg/scan_data.hpp"
#include "liorf/msg/scan_request.hpp"

//third party
#include "SOLiD/solid.h"
#include "fast_max-clique_finder/src/findClique.h"

#include "nabo/nabo.h"
#include "loop_constraint_utils.hpp"
#include "skid_pose_uncertainty.hpp"
#include "skid_comms.hpp"
#include "skid_loop_detection.hpp"
#include "skid_registration.hpp"
#include "skid_registration_params.hpp"

//ros
#include <rclcpp/rclcpp.hpp>
#include "ros_compat.h"
#include "geographic_frames.hpp"


#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>

//gtsam
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

//expression graph
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/slam/expressions.h>
#include <gtsam/slam/dataset.h>

//factor graph
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/ISAM2.h>

//pcl
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <chrono>
#include <limits>
#include <memory>
#include <unordered_map>
#include <thread>
#include <mutex>

inline gtsam::Pose3_ transformTo(const gtsam::Pose3_& x, const gtsam::Pose3_& p) {
    return gtsam::Pose3_(x, &gtsam::Pose3::transformPoseTo, p);
}

class MapFusion : public rclcpp::Node {

private:

    rclcpp::Subscription<liorf::msg::CloudInfo>::SharedPtr _sub_laser_cloud_info;
    rclcpp::Subscription<liorf::msg::ContextInfo>::SharedPtr _sub_solid_info;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _sub_odom_trans;
    rclcpp::Subscription<liorf::msg::LoopConstraint>::SharedPtr _sub_loop_info_global;
    rclcpp::Subscription<liorf::msg::ScanRequest>::SharedPtr _sub_scan_request;
    rclcpp::Subscription<liorf::msg::ScanData>::SharedPtr _sub_scan_data;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_communication_signal;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_signal_1;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_signal_2;

    rclcpp::CallbackGroup::SharedPtr _callback_group;


    std::string _signal_id_1;
    std::string _signal_id_2;

    rclcpp::Publisher<liorf::msg::ContextInfo>::SharedPtr _pub_context_info;
    rclcpp::Publisher<liorf::msg::LoopConstraint>::SharedPtr _pub_loop_info;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_cloud;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _pub_trans_odom2map;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _pub_trans_odom2odom;

    rclcpp::Publisher<liorf::msg::LoopConstraint>::SharedPtr _pub_loop_info_global;
    rclcpp::Publisher<liorf::msg::ScanRequest>::SharedPtr _pub_scan_request;
    rclcpp::Publisher<liorf::msg::ScanData>::SharedPtr _pub_scan_data;

    rclcpp::TimerBase::SharedPtr _comms_maintenance_timer;

    //parameters
    std::string _robot_id;
    std::string _robot_this;//robot id which the thread is now processing
    std::string _solid_topic;
    std::string _loop_topic;
    std::string _solid_frame;
	std::string _map_frame;
	std::string _map_fusion_frame;
	
	std::string _local_topic;

    bool _communication_signal;
    bool _signal_1;
    bool _signal_2;
    bool _use_position_search;

    int _num_bin;
    int _robot_id_th;
    int _robot_this_th;

    int _max_range;
    int _num_sectors;
    int _num_heights;
    int _knn_feature_dim;

    float _fov_up;
    float _fov_down;

    int _num_nearest_matches;
    int _num_match_candidates;

    int _pcm_start_threshold;

    float _loop_thres;
    float _pcm_thres;
    int _loop_frame_thres;
    double _pcm_local_rotation_stddev_rad;
    double _pcm_local_translation_stddev_m;

    liorf::registration::Config _registration_config;

    std::mutex mtx_publish_1;
    std::mutex mtx_publish_2;
    std::mutex mtx;
    // The announcement thread and the callback group both touch the counters.
    mutable std::mutex mtx_stats;

    std::string _robot_initial;
    std::string _pcm_matrix_folder;

    liorf::msg::CloudInfo   _cloud_info;
//    liorf::msg::ContextInfo _context_info;
    // Announcement backlog per peer. Bounded: a peer whose link has been down
    // is better served by recent places than by an unbounded history, and
    // every held entry used to pin a full feature cloud in memory.
    liorf::comms::BoundedQueue<SOLiDBin> _context_list_to_publish_1;
    liorf::comms::BoundedQueue<SOLiDBin> _context_list_to_publish_2;

    // Communication policy: what is retained, what is asked for, and what it
    // all costs. See include/skid_comms.hpp.
    liorf::comms::Config _comms_config;
    bool _announce_scans = false;
    double _announce_rate_hz = 10.0;
    double _comms_maintenance_period_s = 1.0;
    double _comms_report_period_s = 30.0;
    double _last_comms_report_s = 0.0;
    std::unique_ptr<liorf::comms::ScanCache> _scan_cache;
    std::unique_ptr<liorf::comms::RequestTracker> _scan_requests;
    std::unique_ptr<liorf::comms::DeferredCandidateQueue> _deferred_candidates;
    liorf::comms::TransferStats _announce_stats;
    liorf::comms::TransferStats _scan_stats;
    // Every scan this node holds, its own and its peers'. One store with one
    // eviction policy, so the memory ceiling is a single configured number
    // rather than the sum of several unbounded containers.
    std::unordered_map<liorf::comms::ScanKey,
                       pcl::PointCloud<PointType>::Ptr,
                       liorf::comms::ScanKeyHash> _scans;
    // Bin index holding each known descriptor, so an arriving scan can be
    // filed against the place that is already in the KD-tree.
    std::unordered_map<liorf::comms::ScanKey, int, liorf::comms::ScanKeyHash>
        _bin_of_scan_key;
    // Whether this node fuses maps, or only announces its places. Mirrors the
    // condition that decides whether the announcement subscription exists.
    bool _performs_fusion = false;

    pcl::KdTreeFLANN<PointType>::Ptr _kdtree_pose_to_publish;
    pcl::PointCloud<PointType>::Ptr _cloud_pose_to_publish;

    pcl::KdTreeFLANN<PointType>::Ptr _kdtree_pose_to_search;
    pcl::PointCloud<PointType>::Ptr _cloud_pose_to_search_this;//3D
    pcl::PointCloud<PointType>::Ptr _cloud_pose_to_search_other;//3D

    pcl::KdTreeFLANN<PointType>::Ptr _kdtree_loop_to_search;
    pcl::PointCloud<PointType>::Ptr _cloud_loop_to_search;

    std::pair<int, int> _initial_loop;
    int _id_bin_last;

    liorf::msg::LoopConstraint _loop_info;
    std_msgs::msg::Header _cloud_header;

    pcl::PointCloud<PointType>::Ptr _laser_cloud_sum;
    pcl::PointCloud<PointType>::Ptr _laser_cloud_feature;
    pcl::PointCloud<PointType>::Ptr _laser_cloud_corner;
    pcl::PointCloud<PointType>::Ptr _laser_cloud_surface;

    //global variables for solid
    Nabo::NNSearchF* _nns = NULL; //KDtree
    Eigen::MatrixXf _target_matrix;
    SOLiD *_solid_factory;

    std::vector<int> _robot_received_list;
    std::vector<std::pair<int, int>> _idx_nearest_list;
    std::unordered_map<int, SOLiDBin> _bin_with_id;

    struct RegisteredPose {
        gtsam::Pose3 source_pose;
        gtsam::Pose3 aligned_source_pose;
        liorf::uncertainty::Matrix6d covariance =
            liorf::uncertainty::Matrix6d::Zero();
        double truncated_mse_m2 = std::numeric_limits<double>::infinity();
        double overlap_ratio = 0.0;
        std::size_t inliers = 0;
    };

    struct LoopCandidate {
        int target_bin = -1;
        int source_bin = -1;
        gtsam::Pose3 relative_pose;
        liorf::uncertainty::Matrix6d covariance =
            liorf::uncertainty::Matrix6d::Zero();
    };

    struct RegistrationOutput {
        bool valid = false;
        gtsam::Pose3 aligned_source_pose;
        liorf::uncertainty::Matrix6d covariance =
            liorf::uncertainty::Matrix6d::Zero();
        double truncated_mse_m2 = std::numeric_limits<double>::infinity();
        double overlap_ratio = 0.0;
        std::size_t inliers = 0;
    };

    // Registration measurements and corresponding cross-trajectory loops.
    std::unordered_map<int, std::vector<RegisteredPose>> _pose_queue;
    std::unordered_map<int, std::vector<LoopCandidate>> _loop_queue;

    std::unordered_map< std::string, std::vector<PointTypePose> > _global_odom_trans;

    //first: robot pair id, second: effective loop id in _loop_queue
    std::unordered_map< int, std::vector<int> > _loop_accept_queue;

    std::unordered_map< int, std::vector<PointTypePose> > _global_map_trans;
    std::unordered_map< int, PointTypePose> _global_map_trans_optimized;
    // Uncertainty of each optimized map alignment. Composing a cross-peer loop
    // constraint through an alignment treated as exact understates the
    // factor's covariance, so this is carried with it.
    std::unordered_map<int, liorf::uncertainty::Matrix6d>
        _global_map_trans_covariance;
    double _map_alignment_rotation_stddev_rad = 0.05;
    double _map_alignment_translation_stddev_m = 0.20;

    int number_print;
    int _num_cores = 4;

    PointTypePose _trans_to_publish;
    bool _have_trans_to_publish = false;

    std::vector<std::pair<string, double>> _processing_time_list;
public:

    explicit MapFusion(const rclcpp::NodeOptions & options)
    : Node("liorf_mapFusion", options) {
        ParamLoader();
        initialization();

        // ROS 1 spun this node single-threaded; keep callbacks serialised.
        _callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto subOpt = rclcpp::SubscriptionOptions();
        subOpt.callback_group = _callback_group;

        _sub_communication_signal = create_subscription<std_msgs::msg::Bool>(
                _robot_id + "/liorf/signal", rclcpp::QoS(100),
                std::bind(&MapFusion::communicationSignalHandler, this, std::placeholders::_1), subOpt);

        if (!_signal_id_1.empty())
            _sub_signal_1 = create_subscription<std_msgs::msg::Bool>(
                    _signal_id_1 + "/liorf/signal", rclcpp::QoS(100),
                    std::bind(&MapFusion::signalHandler1, this, std::placeholders::_1), subOpt);
        if (!_signal_id_2.empty())
            _sub_signal_2 = create_subscription<std_msgs::msg::Bool>(
                    _signal_id_2 + "/liorf/signal", rclcpp::QoS(100),
                    std::bind(&MapFusion::signalHandler2, this, std::placeholders::_1), subOpt);

        _sub_laser_cloud_info = create_subscription<liorf::msg::CloudInfo>(
                prefixTopic(_robot_id, _local_topic), rclcpp::QoS(1),
                std::bind(&MapFusion::laserCloudInfoHandler, this, std::placeholders::_1), subOpt);

        _sub_loop_info_global = create_subscription<liorf::msg::LoopConstraint>(
                _solid_topic + "/loop_info_global", rclcpp::QoS(100),
                std::bind(&MapFusion::globalLoopInfoHandler, this, std::placeholders::_1), subOpt);


        if(_robot_id != _robot_initial){
            _sub_solid_info = create_subscription<liorf::msg::ContextInfo>(
                _solid_topic + "/context_info", rclcpp::QoS(20),
                std::bind(&MapFusion::solidInfoHandler, this, std::placeholders::_1), subOpt);//number of buffer may differs for different robot numbers
            _sub_odom_trans = create_subscription<nav_msgs::msg::Odometry>(
                _solid_topic + "/trans_odom", rclcpp::QoS(20),
                std::bind(&MapFusion::OdomTransHandler, this, std::placeholders::_1), subOpt);

        }

        // Scan transfer is a separate, on-demand channel from the descriptor
        // announcements above. Every robot both serves and issues requests, so
        // these are subscribed unconditionally.
        _sub_scan_request = create_subscription<liorf::msg::ScanRequest>(
            _solid_topic + "/scan_request", rclcpp::QoS(20),
            std::bind(&MapFusion::scanRequestHandler, this, std::placeholders::_1), subOpt);
        _sub_scan_data = create_subscription<liorf::msg::ScanData>(
            _solid_topic + "/scan_data", rclcpp::QoS(20),
            std::bind(&MapFusion::scanDataHandler, this, std::placeholders::_1), subOpt);



        _pub_context_info = create_publisher<liorf::msg::ContextInfo>(_solid_topic + "/context_info", 1);
        _pub_loop_info = create_publisher<liorf::msg::LoopConstraint>(
                prefixTopic(_robot_id, _loop_topic), 1);
        _pub_cloud = create_publisher<sensor_msgs::msg::PointCloud2>(_robot_id + "/" + _solid_topic + "/cloud", 1);
        _pub_trans_odom2map = create_publisher<nav_msgs::msg::Odometry>(_robot_id + "/" + _solid_topic + "/trans_map", 1);
        _pub_trans_odom2odom = create_publisher<nav_msgs::msg::Odometry>(_solid_topic + "/trans_odom", 1);
        _pub_loop_info_global = create_publisher<liorf::msg::LoopConstraint>(
                _solid_topic + "/loop_info_global", 1);
        _pub_scan_request = create_publisher<liorf::msg::ScanRequest>(
                _solid_topic + "/scan_request", 20);
        _pub_scan_data = create_publisher<liorf::msg::ScanData>(
                _solid_topic + "/scan_data", 20);

        // Resends timed-out scan requests, drops candidates whose scans never
        // arrived, and reports what the link is costing.
        _comms_maintenance_timer = create_wall_timer(
            std::chrono::duration<double>(_comms_maintenance_period_s),
            std::bind(&MapFusion::commsMaintenance, this), _callback_group);

    }

    // Drains one peer's announcement backlog, oldest first.
    //
    // The backlog is FIFO rather than the previous newest-first stack: with a
    // bounded queue, taking from the newest end starves the tail permanently,
    // and dropping is already handled at the bounded queue's oldest end.
    void publishPendingAnnouncements(
        const std::string & peer_id,
        int peer_id_th,
        bool peer_signal,
        std::mutex & queue_mutex,
        liorf::comms::BoundedQueue<SOLiDBin> & queue)
    {
        if (peer_id.empty() || !_communication_signal || !peer_signal ||
            _robot_id_th >= peer_id_th)
            return;

        SOLiDBin bin;
        {
            std::lock_guard<std::mutex> lock(queue_mutex);
            if (!queue.pop(bin))
                return;
        }
        publishContextInfo(bin, peer_id);
    }

    void publishContextInfoThread(){
        int signal_id_th_1 = _signal_id_1.empty() ? -1 : robotID2Number(_signal_id_1);
        int signal_id_th_2 = _signal_id_2.empty() ? -1 : robotID2Number(_signal_id_2);
        // The previous loop had no wait of any kind and spun a core flat out
        // whenever a peer was idle or the backlog was empty.
        rclcpp::Rate rate(_announce_rate_hz);
        while (rclcpp::ok())
        {
            rate.sleep();
            publishPendingAnnouncements(
                _signal_id_1, signal_id_th_1, _signal_1,
                mtx_publish_1, _context_list_to_publish_1);
            publishPendingAnnouncements(
                _signal_id_2, signal_id_th_2, _signal_2,
                mtx_publish_2, _context_list_to_publish_2);
        }
    }

private:
    // Declares a parameter (if not already declared) and returns its value.
    template<typename T>
    T declare_and_get(const std::string & name, const T & default_value)
    {
        if (!this->has_parameter(name))
            this->declare_parameter<T>(name, default_value);
        T value{};
        this->get_parameter(name, value);
        return value;
    }

    void ParamLoader(){
        _robot_id          = declare_and_get<std::string>("robot_id", "jackal0");
        _signal_id_1       = declare_and_get<std::string>("id_1", "jackal1");
        _signal_id_2       = declare_and_get<std::string>("id_2", "jackal2");
        number_print       = declare_and_get<int>("no", 100);
        _pcm_matrix_folder = declare_and_get<std::string>("pcm_matrix_folder", "aaa");

        _knn_feature_dim = declare_and_get<int>("mapfusion.solid.knn_feature_dim", 40);
        _max_range = declare_and_get<int>("mapfusion.solid.max_range", 80);
        _num_sectors = declare_and_get<int>("mapfusion.solid.num_sector", 60);
        _num_heights = declare_and_get<int>("mapfusion.solid.num_height", 64);
        _num_nearest_matches = declare_and_get<int>("mapfusion.solid.num_nearest_matches", 50);
        _num_match_candidates = declare_and_get<int>("mapfusion.solid.num_match_candidates", 1);
        
        _fov_up = declare_and_get<double>("mapfusion.solid.fov_up", 2.0);
        _fov_down = declare_and_get<double>("mapfusion.solid.fov_down", -24.8);

        _loop_thres = declare_and_get<double>("mapfusion.interRobot.loop_threshold", 0.2);
        _pcm_thres = declare_and_get<double>("mapfusion.interRobot.pcm_threshold", 4.0);
        _pcm_local_rotation_stddev_rad = declare_and_get<double>(
            "mapfusion.interRobot.pcm_local_rotation_stddev_rad", 0.05);
        _pcm_local_translation_stddev_m = declare_and_get<double>(
            "mapfusion.interRobot.pcm_local_translation_stddev_m", 0.20);
        // Floor on the map-alignment uncertainty, and the value used outright
        // when the alignment's marginal cannot be recovered. The alignment
        // also carries error this estimator does not model, so it is never
        // treated as better than this.
        _map_alignment_rotation_stddev_rad = declare_and_get<double>(
            "mapfusion.interRobot.map_alignment_rotation_stddev_rad", 0.05);
        _map_alignment_translation_stddev_m = declare_and_get<double>(
            "mapfusion.interRobot.map_alignment_translation_stddev_m", 0.20);
        const double legacy_icp_threshold =
            declare_and_get<double>("mapfusion.interRobot.icp_threshold", 3.0);
        _robot_initial = declare_and_get<std::string>("mapfusion.interRobot.robot_initial", "jackal0");
        _loop_frame_thres = declare_and_get<int>("mapfusion.interRobot.loop_frame_threshold", 10);

        _solid_topic = declare_and_get<std::string>("mapfusion.interRobot.solid_topic", "solid");
        _loop_topic = declare_and_get<std::string>(
            "mapfusion.interRobot.loop_topic", "context/loop_info");
        _solid_frame = declare_and_get<std::string>("mapfusion.interRobot.solid_frame", "base_link");
        _local_topic = declare_and_get<std::string>("mapfusion.interRobot.local_topic", "liorf/mapping/cloud_info");
        _map_frame = liorf::frames::normalizeFrameId(
            declare_and_get<std::string>("liorf.mapFrame", "map"));
        _map_fusion_frame = liorf::frames::normalizeFrameId(
            declare_and_get<std::string>("liorf.mapFusionFrame", ""));
        _pcm_start_threshold = declare_and_get<int>("mapfusion.interRobot.pcm_start_threshold", 5);
        _use_position_search = declare_and_get<bool>("mapfusion.interRobot.use_position_search", false);

        // Communication policy. The descriptor is what the paper exchanges
        // continuously; a scan moves only when a match has already justified
        // registration, and every buffer that holds one is bounded.
        _announce_scans =
            declare_and_get<bool>("mapfusion.comms.announce_scans", false);
        _announce_rate_hz =
            declare_and_get<double>("mapfusion.comms.announce_rate_hz", 10.0);
        _comms_maintenance_period_s = declare_and_get<double>(
            "mapfusion.comms.maintenance_period_s", 1.0);
        _comms_report_period_s = declare_and_get<double>(
            "mapfusion.comms.report_period_s", 30.0);

        const int max_pending_announcements = declare_and_get<int>(
            "mapfusion.comms.max_pending_announcements", 100);
        const int max_cached_scans =
            declare_and_get<int>("mapfusion.comms.max_cached_scans", 500);
        const int max_cached_scan_mib =
            declare_and_get<int>("mapfusion.comms.max_cached_scan_mib", 512);
        const int max_inflight_requests = declare_and_get<int>(
            "mapfusion.comms.max_inflight_requests", 8);
        const int max_request_attempts = declare_and_get<int>(
            "mapfusion.comms.max_request_attempts", 3);
        const int max_deferred_candidates = declare_and_get<int>(
            "mapfusion.comms.max_deferred_candidates", 64);
        if (max_pending_announcements < 1 || max_cached_scans < 1 ||
            max_cached_scan_mib < 1 || max_inflight_requests < 1 ||
            max_request_attempts < 1 || max_deferred_candidates < 1) {
            throw std::invalid_argument(
                "mapfusion.comms bounds must all be at least 1");
        }
        _comms_config.max_pending_announcements =
            static_cast<std::size_t>(max_pending_announcements);
        _comms_config.max_cached_scans =
            static_cast<std::size_t>(max_cached_scans);
        _comms_config.max_cached_scan_bytes =
            static_cast<std::size_t>(max_cached_scan_mib) * 1024u * 1024u;
        _comms_config.max_inflight_requests =
            static_cast<std::size_t>(max_inflight_requests);
        _comms_config.max_request_attempts =
            static_cast<std::size_t>(max_request_attempts);
        _comms_config.max_deferred_candidates =
            static_cast<std::size_t>(max_deferred_candidates);
        _comms_config.request_timeout_s = declare_and_get<double>(
            "mapfusion.comms.request_timeout_s", 5.0);
        _comms_config.max_deferred_age_s = declare_and_get<double>(
            "mapfusion.comms.max_deferred_age_s", 60.0);

        const std::string comms_error = liorf::comms::validate(_comms_config);
        if (!comms_error.empty()) {
            throw std::invalid_argument(
                "invalid mapfusion.comms configuration: " + comms_error);
        }
        if (!std::isfinite(_announce_rate_hz) || _announce_rate_hz <= 0.0 ||
            !std::isfinite(_comms_maintenance_period_s) ||
            _comms_maintenance_period_s <= 0.0 ||
            !std::isfinite(_comms_report_period_s) ||
            _comms_report_period_s <= 0.0) {
            throw std::invalid_argument(
                "mapfusion.comms rates and periods must be finite and positive");
        }

        // Shared with the local mapping node's intra-robot loops so both
        // paths gate on one parameter set. See skid_registration_params.hpp.
        _registration_config = liorf::registration::declareConfig(
            [this](const std::string & name, auto default_value) {
                return this->declare_and_get<decltype(default_value)>(
                    name, default_value);
            },
            "mapfusion.registration.",
            legacy_icp_threshold);

        if (!std::isfinite(_pcm_thres) || _pcm_thres <= 0.0 ||
            _pcm_start_threshold < 2 ||
            !std::isfinite(_pcm_local_rotation_stddev_rad) ||
            _pcm_local_rotation_stddev_rad <= 0.0 ||
            !std::isfinite(_pcm_local_translation_stddev_m) ||
            _pcm_local_translation_stddev_m <= 0.0) {
            throw std::invalid_argument(
                "mapfusion PCM threshold/uncertainty values are invalid");
        }

        if (!std::isfinite(_map_alignment_rotation_stddev_rad) ||
            _map_alignment_rotation_stddev_rad <= 0.0 ||
            !std::isfinite(_map_alignment_translation_stddev_m) ||
            _map_alignment_translation_stddev_m <= 0.0) {
            throw std::invalid_argument(
                "mapfusion map-alignment uncertainty values are invalid");
        }

    }

    void initialization(){
        _laser_cloud_sum.reset(new pcl::PointCloud<PointType>());
        _laser_cloud_feature.reset(new pcl::PointCloud<PointType>());
        // _laser_cloud_corner.reset(new pcl::PointCloud<PointType>());
        _laser_cloud_surface.reset(new pcl::PointCloud<PointType>());

        _solid_factory = new SOLiD(_max_range, _knn_feature_dim, _num_sectors);

        _kdtree_pose_to_publish.reset(new pcl::KdTreeFLANN<PointType>());
        _cloud_pose_to_publish.reset(new pcl::PointCloud<PointType>());

        _kdtree_pose_to_search.reset(new pcl::KdTreeFLANN<PointType>());
        _cloud_pose_to_search_this.reset(new pcl::PointCloud<PointType>());
        _cloud_pose_to_search_other.reset(new pcl::PointCloud<PointType>());

        _kdtree_loop_to_search.reset(new pcl::KdTreeFLANN<PointType>());
        _cloud_loop_to_search.reset(new pcl::PointCloud<PointType>());
		
        _initial_loop.first = -1;

        _robot_id_th = robotID2Number(_robot_id);

        // Mirrors the condition guarding the announcement subscription: the
        // initial robot announces its places but does not fuse maps itself.
        _performs_fusion = (_robot_id != _robot_initial);

        _trans_to_publish.intensity = 0;

        _num_bin = 0;//

        _communication_signal = true;
        _signal_1 = true;
        _signal_2 = true;

        _context_list_to_publish_1 = liorf::comms::BoundedQueue<SOLiDBin>(
            _comms_config.max_pending_announcements);
        _context_list_to_publish_2 = liorf::comms::BoundedQueue<SOLiDBin>(
            _comms_config.max_pending_announcements);
        _scan_cache = std::make_unique<liorf::comms::ScanCache>(_comms_config);
        _scan_requests =
            std::make_unique<liorf::comms::RequestTracker>(_comms_config);
        _deferred_candidates =
            std::make_unique<liorf::comms::DeferredCandidateQueue>(_comms_config);
        _last_comms_report_s = 0.0;

    }

    int robotID2Number(std::string robo){
        const auto suffix = robo.find_last_not_of("0123456789");
        if (suffix == std::string::npos || suffix + 1 >= robo.size())
            throw std::invalid_argument(
                "map-fusion robot IDs must end in a numeric suffix: " + robo);
        return std::stoi(robo.substr(suffix + 1));
    }

    void laserCloudInfoHandler(const liorf::msg::CloudInfo::ConstSharedPtr& msgIn)
    {
        _laser_cloud_sum->clear();
        _laser_cloud_feature->clear();
        // _laser_cloud_corner->clear();
        _laser_cloud_surface->clear();

        //load newest data
        _cloud_info = *msgIn; // new cloud info
        _cloud_header = msgIn->header; // new cloud header
        pcl::fromROSMsg(msgIn->cloud_deskewed, *_laser_cloud_sum);
        // pcl::fromROSMsg(msgIn->cloud_corner, *_laser_cloud_corner);
        pcl::fromROSMsg(msgIn->cloud_surface, *_laser_cloud_surface);
        // *_laser_cloud_feature += *_laser_cloud_corner;
        *_laser_cloud_feature += *_laser_cloud_surface;

        //do solid
        SOLiDBin bin = _solid_factory->ptcloud2bin(_laser_cloud_sum,
                                                   _knn_feature_dim,
                                                   _num_sectors,
                                                   _num_heights,
                                                   _fov_up,
                                                   _fov_down,
                                                   _max_range);
        bin.robotname = _robot_id;
        bin.time = stamp2Sec(_cloud_header.stamp);
        bin.pose.x = _cloud_info.initial_guess_x;
        bin.pose.y = _cloud_info.initial_guess_y;
        bin.pose.z = _cloud_info.initial_guess_z;
        bin.pose.roll  =  _cloud_info.initial_guess_roll;
        bin.pose.pitch =  _cloud_info.initial_guess_pitch;
        bin.pose.yaw   =  _cloud_info.initial_guess_yaw;
        bin.pose.intensity = _cloud_info.imu_available;

        // ptcloud2bin swaps the cloud it is handed into the bin it returns, so
        // bin.cloud aliases _laser_cloud_sum, which the next scan overwrites in
        // place. Anything retained past this callback needs its own buffer.
        bin.cloud.reset(new pcl::PointCloud<PointType>());
        pcl::copyPointCloud(*_laser_cloud_feature,  *bin.cloud);

        // Retain this place's own scan so a peer's request can be answered
        // later. It shares the one bounded budget with received scans.
        storeScan(scanKeyOf(bin), bin.cloud);

        // Peers get the descriptor; the scan stays here until asked for.
        SOLiDBin announcement = bin;
        if (!_announce_scans)
            announcement.cloud.reset(new pcl::PointCloud<PointType>());

        {
            std::lock_guard<std::mutex> lock(mtx_publish_1);
            _context_list_to_publish_1.push(announcement);
        }
        {
            std::lock_guard<std::mutex> lock(mtx_publish_2);
            _context_list_to_publish_2.push(announcement);
        }

        // Own places enter the local index directly. Routing them back through
        // the shared announcement topic would put this robot's own scan on the
        // link for every peer to receive and discard, which is the cost this
        // split exists to remove.
        if (_performs_fusion)
            run(bin);

    }

    void communicationSignalHandler(const std_msgs::msg::Bool::ConstSharedPtr& msg){
        _communication_signal = msg->data;
    }

    void signalHandler1(const std_msgs::msg::Bool::ConstSharedPtr& msg){
        _signal_1 = msg->data;
    }

    void signalHandler2(const std_msgs::msg::Bool::ConstSharedPtr& msg){
        _signal_2 = msg->data;
    }

    double nowSeconds() const { return this->now().seconds(); }

    // The keyframe index a place occupies in its own robot's trajectory.
    //
    // The mapping node transports it in the pose intensity channel, which is a
    // float, so this is exact only below 2^24 keyframes. Every side derives
    // the identity the same way, so the two ends agree regardless.
    static std::int64_t keyframeIndexOf(const SOLiDBin & bin) {
        return static_cast<std::int64_t>(std::llround(bin.pose.intensity));
    }

    static liorf::comms::ScanKey scanKeyOf(const SOLiDBin & bin) {
        liorf::comms::ScanKey key;
        key.robot_id = bin.robotname;
        key.keyframe_index = keyframeIndexOf(bin);
        return key;
    }

    // Payload estimate, not wire size: it counts the point data and the
    // descriptor but not serialization or transport framing.
    static std::size_t cloudBytes(const sensor_msgs::msg::PointCloud2 & cloud) {
        return cloud.data.size();
    }

    std::size_t announcementBytes(const liorf::msg::ContextInfo & info) const {
        return cloudBytes(info.scan_cloud) +
               sizeof(float) * (info.asolid.size() + info.rsolid.size()) +
               info.robot_id.size() + info.robot_id_receive.size() + 64u;
    }

    bool haveScan(const liorf::comms::ScanKey & key) const {
        const auto it = _scans.find(key);
        return it != _scans.end() && it->second && !it->second->empty();
    }

    pcl::PointCloud<PointType>::Ptr findScan(
        const liorf::comms::ScanKey & key) const {
        const auto it = _scans.find(key);
        return it == _scans.end() ? nullptr : it->second;
    }

    // Files a scan and applies the retention policy. The cache decides what
    // must go; this is the only place scans are added or released.
    void storeScan(
        const liorf::comms::ScanKey & key,
        const pcl::PointCloud<PointType>::Ptr & cloud) {
        if (!key.valid() || !cloud || cloud->empty())
            return;

        const std::size_t bytes = cloud->size() * sizeof(PointType);
        _scans[key] = cloud;
        for (const auto & evicted : _scan_cache->insert(key, bytes)) {
            _scans.erase(evicted);
        }
    }

    void publishContextInfo( SOLiDBin bin , std::string robot_to){
        liorf::msg::ContextInfo context_info;
        context_info.robot_id = _robot_id;

        context_info.num_ring = _knn_feature_dim;
        context_info.num_sector = _num_sectors;
        context_info.num_height = _num_heights;

        context_info.asolid.assign(_num_sectors, 0);
        context_info.rsolid.assign(_knn_feature_dim, 0);
        context_info.header = _cloud_header;

        int cnt = 0;

        // SOLiD
        for (int i=0; i<_num_sectors; i++){
            context_info.asolid[i] = bin.asolid(i);
        }
        for (int i=0; i<_knn_feature_dim; i++){
            context_info.rsolid[i] = bin.rsolid(i);
        }

        context_info.robot_id_receive = robot_to;
        context_info.keyframe_index = keyframeIndexOf(bin);
        context_info.pose_x = bin.pose.x;
        context_info.pose_y = bin.pose.y;
        context_info.pose_z = bin.pose.z;
        context_info.pose_roll  =  bin.pose.roll;
        context_info.pose_pitch =  bin.pose.pitch;
        context_info.pose_yaw   =  bin.pose.yaw;
        context_info.pose_intensity = bin.pose.intensity;

        // Announcements are descriptor-only by default. The context topic is
        // shared by every robot, so a cloud attached here is paid for on the
        // link whether or not the addressee is the one that wanted it; the
        // scan follows on request instead.
        if (_announce_scans && bin.cloud) {
            context_info.scan_cloud = publishCloud(
                _pub_cloud, bin.cloud, sec2Stamp(bin.time),
                _robot_id + "/" + _solid_frame);
        }

        const std::size_t bytes = announcementBytes(context_info);
        mtx.lock();
        _pub_context_info->publish(context_info);
        mtx.unlock();
        {
            std::lock_guard<std::mutex> lock(mtx_stats);
            _announce_stats.recordSent(bytes);
        }
    }

    void publishScanRequest(const liorf::comms::ScanKey & key) {
        liorf::msg::ScanRequest request;
        request.header.stamp = this->now();
        request.header.frame_id = _robot_id;
        request.robot_id = _robot_id;
        request.robot_id_receive = key.robot_id;
        request.keyframe_index = key.keyframe_index;
        _pub_scan_request->publish(request);
        {
            std::lock_guard<std::mutex> lock(mtx_stats);
            _scan_stats.recordSent(
                request.robot_id.size() + request.robot_id_receive.size() + 64u);
        }
    }

    // Asks a peer for a scan, subject to the in-flight cap and the retry
    // budget. Returns true when the request is now outstanding.
    bool requestScan(const liorf::comms::ScanKey & key, double now_s) {
        if (key.robot_id == _robot_id) {
            // Our own scan aged out of the cache; no peer can return it.
            RCLCPP_DEBUG(get_logger(),
                "Own scan %s is no longer cached and cannot be recovered",
                key.str().c_str());
            return false;
        }

        const liorf::comms::RequestDecision decision =
            _scan_requests->request(key, now_s);
        switch (decision) {
            case liorf::comms::RequestDecision::kSend:
                publishScanRequest(key);
                return true;
            case liorf::comms::RequestDecision::kAlreadyPending:
                return true;
            default:
                RCLCPP_DEBUG(get_logger(), "Scan request for %s not sent: %s",
                    key.str().c_str(), liorf::comms::toString(decision));
                return false;
        }
    }

    void scanRequestHandler(const liorf::msg::ScanRequest::ConstSharedPtr& msgIn){
        if (!_communication_signal)
            return;
        if (msgIn->robot_id_receive != _robot_id)
            return;

        liorf::comms::ScanKey key;
        key.robot_id = _robot_id;
        key.keyframe_index = msgIn->keyframe_index;

        liorf::msg::ScanData data;
        data.header.stamp = this->now();
        data.header.frame_id = _robot_id;
        data.robot_id = _robot_id;
        data.robot_id_receive = msgIn->robot_id;
        data.keyframe_index = msgIn->keyframe_index;

        const pcl::PointCloud<PointType>::Ptr cloud = findScan(key);
        data.available = cloud && !cloud->empty();
        if (data.available) {
            pcl::toROSMsg(*cloud, data.scan_cloud);
            data.scan_cloud.header.stamp = data.header.stamp;
            data.scan_cloud.header.frame_id = _robot_id + "/" + _solid_frame;
            _scan_cache->touch(key);
        } else {
            // Saying so explicitly stops the requester burning its retry
            // budget on a scan this robot no longer holds.
            RCLCPP_DEBUG(get_logger(),
                "Scan %s requested by %s is no longer held",
                key.str().c_str(), msgIn->robot_id.c_str());
        }

        _pub_scan_data->publish(data);
        {
            std::lock_guard<std::mutex> lock(mtx_stats);
            _scan_stats.recordSent(cloudBytes(data.scan_cloud) + 64u);
        }
    }

    void scanDataHandler(const liorf::msg::ScanData::ConstSharedPtr& msgIn){
        if (!_communication_signal)
            return;
        if (msgIn->robot_id_receive != _robot_id)
            return;

        liorf::comms::ScanKey key;
        key.robot_id = msgIn->robot_id;
        key.keyframe_index = msgIn->keyframe_index;
        if (!key.valid())
            return;

        const double now_s = nowSeconds();
        const double latency = _scan_requests->complete(key, now_s);
        {
            std::lock_guard<std::mutex> lock(mtx_stats);
            _scan_stats.recordReceived(cloudBytes(msgIn->scan_cloud) + 64u);
            if (latency >= 0.0)
                _scan_stats.recordLatency(latency);
        }

        if (!msgIn->available) {
            // The owner cannot supply it, so nothing parked on it can ever
            // complete. Release those candidates rather than let them age out.
            _deferred_candidates->release(key);
            return;
        }

        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
        pcl::fromROSMsg(msgIn->scan_cloud, *cloud);
        if (cloud->empty())
            return;
        storeScan(key, cloud);

        const auto bin_it = _bin_of_scan_key.find(key);
        if (bin_it == _bin_of_scan_key.end()) {
            // The scan arrived before, or without, its descriptor. It is
            // cached; a later announcement will pick it up.
            return;
        }

        bool new_candidate = false;
        for (const auto & ready : _deferred_candidates->release(key)) {
            if (!candidateScansHeld(ready.query_bin, ready.candidate_bin))
                continue;
            new_candidate =
                getInitialGuess(
                    ready.query_bin, ready.candidate_bin, ready.sector_shift) ||
                new_candidate;
        }
        if (new_candidate)
            optimizeAndPublish();
    }

    // Periodic upkeep for the scan channel: resend what timed out, abandon
    // what is past its budget, drop candidates whose scans never arrived, and
    // report what the link is costing.
    void commsMaintenance() {
        const double now_s = nowSeconds();

        const liorf::comms::RequestTracker::Expired expired =
            _scan_requests->expire(now_s);
        for (const auto & key : expired.resend)
            publishScanRequest(key);
        for (const auto & key : expired.abandoned) {
            RCLCPP_WARN(get_logger(),
                "Giving up on scan %s after %zu attempts", key.str().c_str(),
                _comms_config.max_request_attempts);
            _deferred_candidates->release(key);
        }
        _deferred_candidates->expire(now_s);

        if (now_s - _last_comms_report_s < _comms_report_period_s)
            return;
        _last_comms_report_s = now_s;

        std::size_t dropped_1 = 0;
        std::size_t dropped_2 = 0;
        {
            std::lock_guard<std::mutex> lock(mtx_publish_1);
            dropped_1 = _context_list_to_publish_1.dropped();
        }
        {
            std::lock_guard<std::mutex> lock(mtx_publish_2);
            dropped_2 = _context_list_to_publish_2.dropped();
        }
        std::string announce_summary;
        std::string scan_summary;
        {
            std::lock_guard<std::mutex> lock(mtx_stats);
            announce_summary = _announce_stats.summary();
            scan_summary = _scan_stats.summary();
        }

        RCLCPP_INFO(get_logger(),
            "comms announcements: %s | scans: %s | cache %zu scans / %.1f MiB "
            "| requests in flight %zu, retried %zu, abandoned %zu, throttled %zu "
            "| deferred %zu (dropped %zu, expired %zu) "
            "| announcement backlog dropped %zu/%zu | cache evictions %zu",
            announce_summary.c_str(),
            scan_summary.c_str(),
            _scan_cache->size(),
            static_cast<double>(_scan_cache->bytes()) / (1024.0 * 1024.0),
            _scan_requests->inflight(),
            _scan_requests->retried(),
            _scan_requests->abandoned(),
            _scan_requests->throttled(),
            _deferred_candidates->size(),
            _deferred_candidates->dropped(),
            _deferred_candidates->expired(),
            dropped_1,
            dropped_2,
            _scan_cache->evicted());
    }

    void OdomTransHandler(const nav_msgs::msg::Odometry::ConstSharedPtr& odomMsg){
        std::string robot_publish = odomMsg->header.frame_id;
        if( robot_publish == _robot_id)
            return;//skip info publish by the node itself
        std::string robot_child = odomMsg->child_frame_id;
        std::string index = robot_child + robot_publish;
        PointTypePose pose;
        pose.x = odomMsg->pose.pose.position.x;
        pose.y = odomMsg->pose.pose.position.y;
        pose.z = odomMsg->pose.pose.position.z;
        tf2::Quaternion orientation;
        quaternionMsgToTF2(odomMsg->pose.pose.orientation, orientation);
        double roll, pitch, yaw;
        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        pose.roll = roll; pose.pitch = pitch; pose.yaw = yaw;

        //intensity also serves as a index
        pose.intensity = robotID2Number(robot_child);

        auto ite = _global_odom_trans.find(index);
        if(ite == _global_odom_trans.end())//receive a new trans
         {
             std::vector<PointTypePose> tmp_pose_list;
             tmp_pose_list.push_back(pose);
             _global_odom_trans.emplace(std::make_pair(index, tmp_pose_list));
         }
        else//else add a new association;
            _global_odom_trans[index].push_back(pose);

        gtsamFactorGraph();
        sendMapOutputMessage();

    }

    void globalLoopInfoHandler(const liorf::msg::LoopConstraint::ConstSharedPtr& msgIn){
//        return;
        if (msgIn->robot_id != _robot_id)
            return;
        _pub_loop_info->publish(*msgIn);
        sendMapOutputMessage();

    }

    void gtsamFactorGraph(){
        if (_global_map_trans.size() == 0 && _global_odom_trans.size() == 0)
            return;
        gtsam::Vector Vector6(6);
        gtsam::NonlinearFactorGraph graph;
        gtsam::Values initialEstimate;

        std::vector<int> id_received = _robot_received_list;
        std::vector<std::tuple <int, int, gtsam::Pose3>> trans_list;

        if (_global_map_trans.size() != 0)
            id_received.push_back(_robot_id_th);

        //set initial
        Vector6 << 1e-8, 1e-8, 1e-8, 1e-8, 1e-8, 1e-8;
        auto odometryNoise0 = gtsam::noiseModel::Diagonal::Variances(Vector6);
        graph.add( gtsam::PriorFactor<gtsam::Pose3>(gtsam::PriorFactor<gtsam::Pose3>(0, gtsam::Pose3( gtsam::Rot3::RzRyRx(0, 0, 0), gtsam::Point3(0, 0, 0) ) , odometryNoise0) ) );
        initialEstimate.insert(0, gtsam::Pose3( gtsam::Rot3::RzRyRx(0, 0, 0), gtsam::Point3(0, 0, 0)));

        bool ill_posed = true;
        //add local constraints
        for(auto ite : _global_map_trans){
            int id_0 = std::min(ite.first, _robot_id_th);
            int id_1 = std::max(ite.first, _robot_id_th);

            for(auto ite_measure : ite.second){
                PointTypePose pclpose = ite_measure;
                gtsam::Pose3 measurement = gtsam::Pose3 (gtsam::Rot3::RzRyRx(pclpose.roll, pclpose.pitch, pclpose.yaw),
                                                         gtsam::Point3(pclpose.x, pclpose.y, pclpose.z) );
                Vector6 << 1, 1, 1, 1, 1, 1;
                auto odometryNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);
                graph.add( gtsam::BetweenFactor<gtsam::Pose3>(id_0, id_1, measurement, odometryNoise) );
            }

            PointTypePose pclpose = ite.second[ite.second.size() - 1];
            gtsam::Pose3 measurement_latest = gtsam::Pose3 (gtsam::Rot3::RzRyRx(pclpose.roll, pclpose.pitch, pclpose.yaw),
                                                            gtsam::Point3(pclpose.x, pclpose.y, pclpose.z));
            if(id_0 == 0){
                initialEstimate.insert(id_1, measurement_latest);
                ill_posed = false;
            }
            else
                trans_list.emplace_back(std::make_tuple(id_0, id_1, measurement_latest));
        }

        for(auto ite: _global_odom_trans){
            int id_publish = robotID2Number(ite.first);
            int id_child = ite.second[0].intensity;
            int id_0 = std::min(id_publish, id_child);
            int id_1 = std::max(id_publish, id_child);

            for(auto ite_measure: ite.second){

                PointTypePose pclpose = ite_measure;
                gtsam::Pose3 measurement = gtsam::Pose3 (gtsam::Rot3::RzRyRx(pclpose.roll, pclpose.pitch, pclpose.yaw),
                                                         gtsam::Point3(pclpose.x, pclpose.y, pclpose.z));
                Vector6 << 1, 1, 1, 1, 1, 1;
                auto odometryNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);
                graph.add( gtsam::BetweenFactor<gtsam::Pose3>(id_0, id_1, measurement, odometryNoise) );
            }

            PointTypePose pclpose = ite.second[ite.second.size() - 1];
            gtsam::Pose3 measurement_latest = gtsam::Pose3 (gtsam::Rot3::RzRyRx(pclpose.roll, pclpose.pitch, pclpose.yaw),
                                                     gtsam::Point3(pclpose.x, pclpose.y, pclpose.z));

            if(id_0 == 0){
                initialEstimate.insert(id_1, measurement_latest);
                ill_posed = false;
            }
            else
                trans_list.emplace_back(std::make_tuple(id_0, id_1, measurement_latest));

            if(find(id_received.begin(), id_received.end(), id_0) == id_received.end())
                id_received.push_back(id_0);

            if(find(id_received.begin(), id_received.end(), id_1) == id_received.end())
                id_received.push_back(id_1);

        }

        if (find(id_received.begin(), id_received.end(), _robot_id_th) == id_received.end()){
            return;
        }
        if (ill_posed)
            return;

        bool terminate_signal = false;
        while (!terminate_signal){
            if (id_received.size() == 0)
                break;
            terminate_signal = true;
            for(auto id = id_received.begin(); id != id_received.end();)
            {
                int id_this = *id;
                if(initialEstimate.exists(id_this)){
                    id = id_received.erase(id);
                    continue;
                }
                else
                    ++id;

                auto it = std::find_if(trans_list.begin(), trans_list.end(),
                                       [id_this](auto& e)
                                       {return std::get<0>(e) == id_this || std::get<1>(e) == id_this;});

                if(it == trans_list.end())
                    continue;

                int id_t = get<0>(*it) + get<1>(*it) - id_this;

                if(!initialEstimate.exists(id_t))
                    continue;
                gtsam::Pose3 pose_t = initialEstimate.at<gtsam::Pose3>(id_t);
                if ( id_this == get<1>(*it)){
                    gtsam::Pose3 pose_f = pose_t * get<2>(*it);
                    initialEstimate.insert(id_this, pose_f);
                    terminate_signal = false;

                }
                if ( id_this == get<0>(*it)){
                    gtsam::Pose3 pose_f = pose_t * get<2>(*it).inverse();
                    initialEstimate.insert(id_this, pose_f);
                    terminate_signal = false;
                }
            }
        }

        for (auto it : id_received){
            initialEstimate.insert(it, gtsam::Pose3( gtsam::Rot3::RzRyRx(0, 0, 0), gtsam::Point3(0, 0, 0) ));
        }

        id_received.clear();
        trans_list.clear();

        gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initialEstimate).optimize();

        initialEstimate.clear();
        graph.resize(0);

        gtsam::Pose3 est = result.at<gtsam::Pose3>(_robot_id_th);

        _trans_to_publish.x = est.translation().x();
        _trans_to_publish.y = est.translation().y();
        _trans_to_publish.z = est.translation().z();
        _trans_to_publish.roll = est.rotation().roll();
        _trans_to_publish.pitch = est.rotation().pitch();
        _trans_to_publish.yaw = est.rotation().yaw();
        _trans_to_publish.intensity = 1;
        _have_trans_to_publish = true;

        if (_have_trans_to_publish){
            int robot_id_initial = robotID2Number(_robot_initial);
            if (_global_map_trans_optimized.find(robot_id_initial) == _global_map_trans_optimized.end()){
                _global_map_trans_optimized.emplace(std::make_pair(robot_id_initial, _trans_to_publish));
            }


            else{
                _global_map_trans[robot_id_initial].push_back(_trans_to_publish);
                _global_map_trans_optimized[robot_id_initial] = _trans_to_publish;
                }
        }
    }

    void solidInfoHandler(const liorf::msg::ContextInfo::ConstSharedPtr& msgIn){
        liorf::msg::ContextInfo context_info_input = *msgIn;
        //load the data received
        if (!_communication_signal)
            return;
        if (msgIn->robot_id_receive != _robot_id)
            return;

        SOLiDBin bin;
        bin.robotname = msgIn->robot_id;
        bin.time = stamp2Sec(msgIn->header.stamp);

        bin.pose.x = msgIn->pose_x;
        bin.pose.y = msgIn->pose_y;
        bin.pose.z = msgIn->pose_z;
        bin.pose.roll  = msgIn->pose_roll;
        bin.pose.pitch = msgIn->pose_pitch;
        bin.pose.yaw   = msgIn->pose_yaw;
        // The explicit keyframe index is the place's identity; pose_intensity
        // carries the same value for the legacy consumers of this message.
        bin.pose.intensity = static_cast<float>(msgIn->keyframe_index);

        // An announcement may or may not carry its scan. When it does the
        // scan is filed immediately; when it does not, it is requested only if
        // a descriptor match turns into a candidate.
        bin.cloud.reset(new pcl::PointCloud<PointType>());
        if (!msgIn->scan_cloud.data.empty())
            pcl::fromROSMsg(msgIn->scan_cloud, *bin.cloud);
        {
            std::lock_guard<std::mutex> lock(mtx_stats);
            _announce_stats.recordReceived(announcementBytes(*msgIn));
        }

        // SOLiD
        bin.asolid = Eigen::VectorXf::Zero(_num_sectors);
        bin.rsolid = Eigen::VectorXf::Zero(_knn_feature_dim);
        int cnt = 0;
        for (int i=0; i<msgIn->num_sector; i++){
            bin.asolid(i) = msgIn->asolid[i];
        }
        for (int i=0; i<msgIn->num_ring; i++){
            bin.rsolid(i) = msgIn->rsolid[i];
        }

        run(bin);

    }

    void run(SOLiDBin bin){
        //build

        buildKDTree(bin);
        KNNSearch(bin);

        if(_idx_nearest_list.empty() && _use_position_search)
            distanceSearch(bin);

        if(!getInitialGuesses()){
            return;
        }

        optimizeAndPublish();
    }

    // Runs once new inter-robot candidates exist, whether they were produced
    // by an arriving announcement or by a scan that completed a parked
    // candidate later.
    void optimizeAndPublish(){
        if(!incrementalPCM()){
            return;
        }

        //perform optimization
        gtsamExpressionGraph();

        //send out transform
        sendOdomOutputMessage();
    }

    void distanceSearch(SOLiDBin bin){
        int id_this = robotID2Number(bin.robotname);
        if(bin.robotname != _robot_id){
            if(_global_map_trans_optimized.find(id_this) == _global_map_trans_optimized.end() )
                return;
            PointType pt_query;
            std::vector<int> idx_list;
            std::vector<float> dist_list;
            PointTypePose pose_this = _global_map_trans_optimized[id_this];
            gtsam::Pose3 T_pose_this = gtsam::Pose3(gtsam::Rot3::RzRyRx(pose_this.roll, pose_this.pitch, pose_this.yaw),
                            gtsam::Point3(pose_this.x, pose_this.y, pose_this.z));
            auto T_query = gtsam::Pose3(gtsam::Rot3::RzRyRx(bin.pose.roll, bin.pose.pitch, bin.pose.yaw),
                                        gtsam::Point3(bin.pose.x, bin.pose.y, bin.pose.z));
            T_query = T_pose_this.inverse() * T_query;

            pt_query.x = T_query.translation().x(); pt_query.y = T_query.translation().y(); pt_query.z = T_query.translation().z();

            _kdtree_pose_to_search->setInputCloud(_cloud_pose_to_search_this);
            _kdtree_pose_to_search->radiusSearch(pt_query, 5, idx_list, dist_list, 0);
            if (!idx_list.empty()){
                int tmp_id = _cloud_pose_to_search_this->points[idx_list[0]].intensity;
                _idx_nearest_list.emplace_back(std::make_pair(tmp_id, 0));
            }
        }
        else{
            PointType pt_query;
            std::vector<int> idx_list;
            std::vector<float> dist_list;
            pcl::PointCloud<PointType>::Ptr cloud_pose_to_search_other_copy(new pcl::PointCloud<PointType>());
            for (unsigned int i = 0; i < _cloud_pose_to_search_other->size(); i++){
                PointType tmp = _cloud_pose_to_search_other->points[i];
                int id_this = robotID2Number(_bin_with_id[tmp.intensity].robotname);
                if(_global_map_trans_optimized.find(id_this) == _global_map_trans_optimized.end() )
                    continue;
                PointTypePose pose_this = _global_map_trans_optimized[id_this];
                gtsam::Pose3 T_pose_this = gtsam::Pose3(gtsam::Rot3::RzRyRx(pose_this.roll, pose_this.pitch, pose_this.yaw),
                                                        gtsam::Point3(pose_this.x, pose_this.y, pose_this.z));
                auto T_this = gtsam::Point3(tmp.x,tmp.y,tmp.z);
                T_this = T_pose_this.inverse() * T_this;
                tmp.x = T_this.x();
                tmp.y = T_this.y();
                tmp.z = T_this.z();
                cloud_pose_to_search_other_copy->push_back(tmp);
            }
            pt_query.x = bin.pose.x;
            pt_query.y = bin.pose.y;
            pt_query.z = bin.pose.z;
            if (cloud_pose_to_search_other_copy->empty())
                return;

            _kdtree_pose_to_search->setInputCloud(cloud_pose_to_search_other_copy);
            _kdtree_pose_to_search->radiusSearch(pt_query, 10, idx_list, dist_list, 0);
            if (!idx_list.empty()){
                for (unsigned int i = 0; i< cloud_pose_to_search_other_copy->size(); i++){
                    int tmp_id = cloud_pose_to_search_other_copy->points[idx_list[i]].intensity;
                    if(tmp_id == _num_bin)
                        continue;
                    _idx_nearest_list.emplace_back(std::make_pair(tmp_id, 0));
                    break;
                }
            }
            cloud_pose_to_search_other_copy->clear();

        }//


    }

    void buildKDTree(SOLiDBin bin){
        _num_bin++;

        // The scan store owns point clouds; the bin keeps the descriptor and
        // pose. Keeping both would give one cloud two lifetimes, only one of
        // which the retention policy controls.
        const liorf::comms::ScanKey key = scanKeyOf(bin);
        if (bin.cloud && !bin.cloud->empty())
            storeScan(key, bin.cloud);
        bin.cloud.reset(new pcl::PointCloud<PointType>());

        //store data received
        _bin_with_id.emplace( std::make_pair(_num_bin-1, bin) );
        if (key.valid())
            _bin_of_scan_key[key] = _num_bin - 1;

        PointType tmp_pose;
        tmp_pose.x = bin.pose.x; tmp_pose.y = bin.pose.y; tmp_pose.z = bin.pose.z;
        tmp_pose.intensity = _num_bin - 1;

        if (bin.robotname == _robot_id)
            _cloud_pose_to_search_this->push_back(tmp_pose);
        else
            _cloud_pose_to_search_other->push_back(tmp_pose);

        //add the latest ringkey
        _target_matrix.conservativeResize(_knn_feature_dim, _num_bin);

        // For solid
        Eigen::VectorXf ringkey_segment = bin.rsolid.block(0, 0, _knn_feature_dim, 1);
        float norm = ringkey_segment.norm();
        if (norm != 0) {
            ringkey_segment /= norm; // 벡터를 노름으로 나누어 정규화
        }
        _target_matrix.block(0, _num_bin-1, _knn_feature_dim, 1) = ringkey_segment;

        _nns = Nabo::NNSearchF::createKDTreeLinearHeap(_target_matrix);
    }

    void KNNSearch(SOLiDBin bin){
        if (_num_nearest_matches >= _num_bin){
            return;//if not enough candidates, return
        }

        int num_neighbors = _num_nearest_matches;

        //search n nearest neighbors
        Eigen::VectorXi indices(num_neighbors);
        Eigen::VectorXf dists2(num_neighbors);

        Eigen::VectorXf ringkey_segment = bin.rsolid.block(0, 0, _knn_feature_dim, 1);
        float norm = ringkey_segment.norm();
        if (norm != 0) {
            ringkey_segment /= norm; // 벡터를 노름으로 나누어 정규화
        }
        _nns->knn(ringkey_segment, indices, dists2, num_neighbors);

        int idx_candidate, rot_idx;
        float distance_to_query;
        //first: dist, second: idx in bin, third: rot_idx
        std::vector<std::tuple<float, int, int>> idx_list;
        for (int i = 0; i < std::min( num_neighbors, int(indices.size()) ); ++i){
            //check if the searching work normally
            if ( indices.sum() == 0)
                continue;

            idx_candidate = indices[i];
            if ( idx_candidate >= _num_bin)
                continue;

            // if the candidate & source belong to same robot, skip
            if ( bin.robotname == _bin_with_id.at(idx_candidate).robotname)
                continue;

            // if the matching pair have nothing to do with the present robot, skip
            if ( bin.robotname != _robot_id && _bin_with_id.at(idx_candidate).robotname != _robot_id)
                continue;

            if( robotID2Number(bin.robotname) >= _robot_id_th && robotID2Number(_bin_with_id.at(idx_candidate).robotname) >= _robot_id_th)
                continue;

            //compute the dist with full solid info
            distance_to_query = distBtnSOLiDs(bin.rsolid, _bin_with_id.at(idx_candidate).rsolid, 
                                              bin.asolid, _bin_with_id.at(idx_candidate).asolid, rot_idx);

            if( distance_to_query > _loop_thres)
                continue;

            //add to idx list
            idx_list.emplace_back( std::make_tuple(distance_to_query, idx_candidate, rot_idx) );
        }

        _idx_nearest_list.clear();

        if (idx_list.size() == 0)
            return;

        //find nearest solids
        std::sort(idx_list.begin(), idx_list.end());
        for (int i = 0; i < std::min( _num_match_candidates, int(idx_list.size()) ); i++){
            std::tie(distance_to_query, idx_candidate, rot_idx) = idx_list[i];
            _idx_nearest_list.emplace_back(std::make_pair(idx_candidate, rot_idx));
        }
        idx_list.clear();
    }

    // True when both scans behind a candidate are held right now.
    bool candidateScansHeld(int query_bin, int candidate_bin) const {
        const auto query = _bin_with_id.find(query_bin);
        const auto candidate = _bin_with_id.find(candidate_bin);
        if (query == _bin_with_id.end() || candidate == _bin_with_id.end())
            return false;
        return haveScan(scanKeyOf(query->second)) &&
               haveScan(scanKeyOf(candidate->second));
    }

    // Ensures both scans behind a candidate are available, asking for what is
    // missing and parking the candidate until it arrives.
    //
    // Returns true only when the candidate can be registered immediately.
    bool ensureCandidateScans(int query_bin, int candidate_bin, int min_idx,
                              double now_s) {
        const auto query = _bin_with_id.find(query_bin);
        const auto candidate = _bin_with_id.find(candidate_bin);
        if (query == _bin_with_id.end() || candidate == _bin_with_id.end())
            return false;

        std::vector<liorf::comms::ScanKey> missing;
        for (const SOLiDBin * bin : {&query->second, &candidate->second}) {
            const liorf::comms::ScanKey key = scanKeyOf(*bin);
            if (!key.valid())
                return false;
            if (haveScan(key)) {
                _scan_cache->touch(key);
                continue;
            }
            missing.push_back(key);
        }
        if (missing.empty())
            return true;

        bool every_request_outstanding = true;
        for (const auto & key : missing)
            every_request_outstanding =
                requestScan(key, now_s) && every_request_outstanding;
        if (!every_request_outstanding) {
            // Nothing will arrive for at least one of these scans, so parking
            // the candidate would only occupy a slot until it aged out.
            return false;
        }

        liorf::comms::DeferredCandidate deferred;
        deferred.query_bin = query_bin;
        deferred.candidate_bin = candidate_bin;
        deferred.sector_shift = min_idx;
        deferred.parked_at_s = now_s;
        deferred.missing = std::move(missing);
        _deferred_candidates->park(std::move(deferred));
        return false;
    }

    bool getInitialGuesses(){
        if(_idx_nearest_list.size() == 0){
            return false;
        }
        const int query_bin = _num_bin - 1;
        const double now_s = nowSeconds();
        bool new_candidate_signal = false;
        for (auto it: _idx_nearest_list){
            if (!ensureCandidateScans(query_bin, it.first, it.second, now_s))
                continue;
            // Evaluated first so every candidate is attempted, not just the
            // last one: the previous assignment discarded earlier results.
            new_candidate_signal =
                getInitialGuess(query_bin, it.first, it.second) ||
                new_candidate_signal;
        }
        return new_candidate_signal;
    }

    bool getInitialGuess(int query_bin, int idx_nearest, int min_idx){

        const auto query_it = _bin_with_id.find(query_bin);
        if (query_it == _bin_with_id.end())
            return false;
        SOLiDBin bin = query_it->second;

        int id0 = idx_nearest, id1 = query_bin;

        SOLiDBin bin_nearest;
        PointTypePose source_pose_initial, target_pose;

        // Azimuth rotation implied by the winning A-SOLiD shift. The shared
        // helper counts whole sectors; the previous local expression added one
        // extra sector of yaw to every coarse guess.
        float solid_pitch =
            liorf::loop_detection::sectorShiftToYaw(min_idx, _num_sectors);

        int robot_id_this = robotID2Number(bin.robotname);

        auto robot_id_this_ite = std::find(_robot_received_list.begin(),  _robot_received_list.end(), robot_id_this);

        //record all received robot id from other robots
        if (robot_id_this_ite == _robot_received_list.end() && robot_id_this != _robot_id_th)
            _robot_received_list.push_back(robot_id_this);

        //  exchange if source has a prior robot id (the last character of the robot name is smaller) (first > second)
        if (robot_id_this < _robot_id_th){
            bin_nearest = bin;
            bin = _bin_with_id.at(idx_nearest);

            id0 = query_bin;
            id1 = idx_nearest;

            solid_pitch = -solid_pitch;
        }
        else
            bin_nearest = _bin_with_id.at(idx_nearest);

        _robot_this = bin_nearest.robotname;
        _robot_this_th = robotID2Number(_robot_this);

        //get initial guess from solid
        target_pose = bin_nearest.pose;

        //find the pose constrain
        if (_global_map_trans_optimized.find(_robot_this_th) != _global_map_trans_optimized.end()){
            PointTypePose trans_to_that = _global_map_trans_optimized[_robot_this_th];
            Eigen::Affine3f t_source2target = pcl::getTransformation(trans_to_that.x, trans_to_that.y, trans_to_that.z,
                                                               trans_to_that.roll, trans_to_that.pitch, trans_to_that.yaw);
            Eigen::Affine3f t_source = pcl::getTransformation(bin.pose.x, bin.pose.y, bin.pose.z, bin.pose.roll, bin.pose.pitch, bin.pose.yaw);
            Eigen::Affine3f t_initial_source = t_source2target * t_source;
            pcl::getTranslationAndEulerAngles(t_initial_source, source_pose_initial.x, source_pose_initial.y, source_pose_initial.z,
                                              source_pose_initial.roll, source_pose_initial.pitch, source_pose_initial.yaw);
            //if too far away, return false

        }
        else if(abs(solid_pitch) < 0.3){
            source_pose_initial = target_pose;
        }
        else{
            Eigen::Affine3f solid_initial = pcl::getTransformation(0, 0, 0,
                                                                0, 0, solid_pitch);
            Eigen::Affine3f t_target = pcl::getTransformation(target_pose.x, target_pose.y, target_pose.z,
                                                              target_pose.roll, target_pose.pitch, target_pose.yaw);
            Eigen::Affine3f t_initial_source =  solid_initial * t_target;
            // pre-multiplying -> successive rotation about a fixed frame

            pcl::getTranslationAndEulerAngles(t_initial_source, source_pose_initial.x, source_pose_initial.y, source_pose_initial.z,
                                              source_pose_initial.roll, source_pose_initial.pitch, source_pose_initial.yaw);
            source_pose_initial.x =  target_pose.x;
            source_pose_initial.y =  target_pose.y;
            source_pose_initial.z =  target_pose.z;
        }

        const pcl::PointCloud<PointType>::Ptr source_scan =
            findScan(scanKeyOf(bin));
        const pcl::PointCloud<PointType>::Ptr target_scan =
            findScan(scanKeyOf(bin_nearest));
        if (!source_scan || source_scan->empty() ||
            !target_scan || target_scan->empty()) {
            // Evicted between being requested and being used.
            RCLCPP_DEBUG(get_logger(),
                "Candidate %d -> %d dropped: a scan is no longer held",
                id0, id1);
            return false;
        }

        const RegistrationOutput registered = registerRelativeMotion(
            transformPointCloud(source_scan, &source_pose_initial),
            transformPointCloud(target_scan, &target_pose),
            source_pose_initial);

        if (!registered.valid)
            return false;

        //1: jackal0, 2: jackal1
        gtsam::Pose3 pose_from =
            gtsam::Pose3(gtsam::Rot3::RzRyRx(bin.pose.roll, bin.pose.pitch, bin.pose.yaw),
                  gtsam::Point3(bin.pose.x, bin.pose.y, bin.pose.z));

        gtsam::Pose3 pose_target =
            gtsam::Pose3(gtsam::Rot3::RzRyRx(target_pose.roll, target_pose.pitch, target_pose.yaw),
                  gtsam::Point3(target_pose.x, target_pose.y, target_pose.z));

        const auto relative_to_target = liorf::uncertainty::between(
            registered.aligned_source_pose, registered.covariance,
            pose_target, liorf::uncertainty::Matrix6d::Zero());
        if (!liorf::uncertainty::validCovariance(relative_to_target.covariance)) {
            RCLCPP_WARN(get_logger(),
                "SKiD registration covariance failed relative-pose propagation");
            return false;
        }

        _pose_queue[_robot_this_th].push_back(RegisteredPose{
            pose_from,
            registered.aligned_source_pose,
            registered.covariance,
            registered.truncated_mse_m2,
            registered.overlap_ratio,
            registered.inliers});
        _loop_queue[_robot_this_th].push_back(LoopCandidate{
            id0, id1, relative_to_target.pose, relative_to_target.covariance});

        return true;
    }

    // Inter- and intra-robot loops score places with the same shared SOLiD
    // distance; see include/skid_loop_detection.hpp.
    float distBtnSOLiDs(Eigen::VectorXf rsolid1, Eigen::VectorXf rsolid2, Eigen::VectorXf asolid1, Eigen::VectorXf asolid2, int & idx){
        liorf::loop_detection::Descriptor query;
        query.range = std::move(rsolid1);
        query.angular = std::move(asolid1);
        liorf::loop_detection::Descriptor candidate;
        candidate.range = std::move(rsolid2);
        candidate.angular = std::move(asolid2);

        const liorf::loop_detection::Comparison comparison =
            liorf::loop_detection::compare(query, candidate);
        idx = comparison.sector_shift;
        // An unusable descriptor pair scores worse than any threshold rather
        // than producing a NaN that compares false against every gate.
        return comparison.valid() ? comparison.range_distance
                                  : std::numeric_limits<float>::infinity();
    }
    
    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        PointType *pointFrom;

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

        Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);

#pragma omp parallel for num_threads(_num_cores)
        for (int i = 0; i < cloudSize; ++i)
        {
            pointFrom = &cloudIn->points[i];
            cloudOut->points[i].x = transCur(0,0) * pointFrom->x + transCur(0,1) * pointFrom->y + transCur(0,2) * pointFrom->z + transCur(0,3);
            cloudOut->points[i].y = transCur(1,0) * pointFrom->x + transCur(1,1) * pointFrom->y + transCur(1,2) * pointFrom->z + transCur(1,3);
            cloudOut->points[i].z = transCur(2,0) * pointFrom->x + transCur(2,1) * pointFrom->y + transCur(2,2) * pointFrom->z + transCur(2,3);
            cloudOut->points[i].intensity = pointFrom->intensity;
        }
        return cloudOut;
    }

    RegistrationOutput registerRelativeMotion(
                                         pcl::PointCloud<PointType>::Ptr source,
                                         pcl::PointCloud<PointType>::Ptr target,
                                         const PointTypePose& pose_source)
    {
        RegistrationOutput output;

        liorf::registration::PointCloud source_points;
        liorf::registration::PointCloud target_points;
        source_points.reserve(source->size());
        target_points.reserve(target->size());
        for (const auto& point : source->points) {
            source_points.emplace_back(point.x, point.y, point.z);
        }
        for (const auto& point : target->points) {
            target_points.emplace_back(point.x, point.y, point.z);
        }

        const liorf::registration::Result registration =
            liorf::registration::registerClouds(
                source_points, target_points, _registration_config);
        if (!registration.accepted()) {
            RCLCPP_WARN(
                get_logger(),
                "SKiD registration rejected (%s): %s "
                "[corr=%zu, coarse_inliers=%zu, fine_inliers=%zu, overlap=%.3f, tMSE=%.6f m^2]",
                liorf::registration::toString(registration.status),
                registration.detail.c_str(),
                registration.coarse_correspondences,
                registration.coarse_translation_inliers,
                registration.fine_inliers,
                registration.metric.overlap_ratio,
                registration.metric.value_m2);
            return output;
        }

        const gtsam::Pose3 correction_target_from_source(
            gtsam::Rot3(registration.T_target_source.linear()),
            gtsam::Point3(registration.T_target_source.translation()));
        const gtsam::Pose3 initial_world_from_lidar(
            gtsam::Rot3::RzRyRx(
                pose_source.roll, pose_source.pitch, pose_source.yaw),
            gtsam::Point3(pose_source.x, pose_source.y, pose_source.z));
        const auto aligned = liorf::uncertainty::compose(
            correction_target_from_source,
            registration.uncertainty.covariance,
            initial_world_from_lidar,
            liorf::uncertainty::Matrix6d::Zero());
        if (!liorf::uncertainty::validCovariance(aligned.covariance)) {
            RCLCPP_WARN(get_logger(),
                "SKiD registration covariance failed pose-composition propagation");
            return output;
        }

        output.valid = true;
        output.aligned_source_pose = aligned.pose;
        output.covariance = aligned.covariance;
        output.truncated_mse_m2 = registration.metric.value_m2;
        output.overlap_ratio = registration.metric.overlap_ratio;
        output.inliers = registration.metric.correspondence_count;

        RCLCPP_INFO(
            get_logger(),
            "SKiD registration accepted: corr=%zu coarse_inliers=%zu "
            "fine_inliers=%zu overlap=%.3f tMSE=%.6f m^2 "
            "uncertainty_scale=%.2f condition=%.1f clamped_modes=%zu "
            "time=%.1f ms (coarse %.1f, fine %.1f)",
            registration.coarse_correspondences,
            registration.coarse_translation_inliers,
            registration.fine_inliers,
            registration.metric.overlap_ratio,
            registration.metric.value_m2,
            registration.uncertainty.variance_scale,
            registration.uncertainty.condition_number,
            registration.uncertainty.clamped_modes,
            1000.0 * (registration.coarse_seconds + registration.fine_seconds +
                      registration.metric_seconds),
            1000.0 * registration.coarse_seconds,
            1000.0 * registration.fine_seconds);

        return output;

    }

    bool incrementalPCM() {
        if (_pose_queue[_robot_this_th].size() <
            static_cast<std::size_t>(_pcm_start_threshold))
            return false;

        //perform pcm for all robot matches

        Eigen::MatrixXi consistency_matrix = computePCMMatrix(_loop_queue[_robot_this_th]);//, _pose_queue[_robot_this_th]);
        std::string consistency_matrix_file = _pcm_matrix_folder + "/consistency_matrix" + _robot_id + ".clq.mtx";
        printPCMGraph(consistency_matrix, consistency_matrix_file);
        // Compute maximum clique
        FMC::CGraphIO gio;
        gio.readGraph(consistency_matrix_file);
        std::vector<int> max_clique_data;

        FMC::maxCliqueHeu(gio, max_clique_data);

        std::sort(max_clique_data.begin(), max_clique_data.end());

        auto loop_accept_queue_this = _loop_accept_queue.find(_robot_this_th);
        if (loop_accept_queue_this == _loop_accept_queue.end()){
            _loop_accept_queue.emplace(std::make_pair(_robot_this_th, max_clique_data));
            return true;
        }

        if(max_clique_data == loop_accept_queue_this->second)
            return false;

        _loop_accept_queue[_robot_this_th].clear();
        _loop_accept_queue[_robot_this_th] = max_clique_data;
        return true;
    }

    Eigen::MatrixXi computePCMMatrix(const std::vector<LoopCandidate>& loop_queue_this){
        Eigen::MatrixXi PCMMat;
        PCMMat.setZero(loop_queue_this.size(), loop_queue_this.size());
        gtsam::Pose3 z_ai_aj, z_bk_bl;
        gtsam::Pose3 t_ai, t_aj, t_bk, t_bl;

        for (unsigned int i = 0; i < loop_queue_this.size(); i++){
            const auto& first_loop = loop_queue_this[i];
            PointTypePose tmp_pose_0 = _bin_with_id.at(first_loop.target_bin).pose;
            PointTypePose tmp_pose_1 = _bin_with_id.at(first_loop.source_bin).pose;
            t_aj = gtsam::Pose3(gtsam::Rot3::RzRyRx(tmp_pose_0.roll, tmp_pose_0.pitch, tmp_pose_0.yaw),
                                gtsam::Point3(tmp_pose_0.x, tmp_pose_0.y, tmp_pose_0.z));
            t_bk = gtsam::Pose3(gtsam::Rot3::RzRyRx(tmp_pose_1.roll, tmp_pose_1.pitch, tmp_pose_1.yaw),
                                gtsam::Point3(tmp_pose_1.x, tmp_pose_1.y, tmp_pose_1.z));

            for (unsigned int j = i + 1; j < loop_queue_this.size(); j++){
                const auto& second_loop = loop_queue_this[j];
                PointTypePose tmp_pose_0 = _bin_with_id.at(second_loop.target_bin).pose;
                PointTypePose tmp_pose_1 = _bin_with_id.at(second_loop.source_bin).pose;
                t_ai = gtsam::Pose3(gtsam::Rot3::RzRyRx(tmp_pose_0.roll, tmp_pose_0.pitch, tmp_pose_0.yaw),
                                    gtsam::Point3(tmp_pose_0.x, tmp_pose_0.y, tmp_pose_0.z));
                t_bl = gtsam::Pose3(gtsam::Rot3::RzRyRx(tmp_pose_1.roll, tmp_pose_1.pitch, tmp_pose_1.yaw),
                                    gtsam::Point3(tmp_pose_1.x, tmp_pose_1.y, tmp_pose_1.z));
                z_ai_aj = t_ai.between(t_aj);
                z_bk_bl = t_bk.between(t_bl);
                const auto residual = liorf::uncertainty::pcmResidual(
                    first_loop.relative_pose, first_loop.covariance,
                    second_loop.relative_pose, second_loop.covariance,
                    z_ai_aj, z_bk_bl,
                    _pcm_local_rotation_stddev_rad,
                    _pcm_local_translation_stddev_m);
                if (residual.valid &&
                    residual.mahalanobis_distance < _pcm_thres)
                    PCMMat(i,j) = 1;
                else
                    PCMMat(i,j) = 0;
            }
        }
        return PCMMat;
    }

    void printPCMGraph(Eigen::MatrixXi pcm_matrix, std::string file_name) {
        // Intialization
        int nb_consistent_measurements = 0;

        // Format edges.
        std::stringstream ss;
        for (int i = 0; i < pcm_matrix.rows(); i++) {
            for (int j = i; j < pcm_matrix.cols(); j++) {
                if (pcm_matrix(i,j) == 1) {
                    ss << i+1 << " " << j+1 << std::endl;
                    nb_consistent_measurements++;
                }
            }
        }

        // Write to file
        std::ofstream output_file;
        output_file.open(file_name);
        output_file << "%%MatrixMarket matrix coordinate pattern symmetric" << std::endl;
        output_file << pcm_matrix.rows() << " " << pcm_matrix.cols() << " " << nb_consistent_measurements << std::endl;
        output_file << ss.str();
        output_file.close();
    }

    liorf::uncertainty::Matrix6d alignmentUncertaintyFloor() const {
        Eigen::Matrix<double, 6, 1> variances;
        const double rotation_variance =
            _map_alignment_rotation_stddev_rad *
            _map_alignment_rotation_stddev_rad;
        const double translation_variance =
            _map_alignment_translation_stddev_m *
            _map_alignment_translation_stddev_m;
        // GTSAM tangent order: [rx, ry, rz, tx, ty, tz].
        variances << rotation_variance, rotation_variance, rotation_variance,
            translation_variance, translation_variance, translation_variance;
        return liorf::uncertainty::Matrix6d(variances.asDiagonal());
    }

    struct MapAlignment {
        gtsam::Pose3 pose;
        liorf::uncertainty::Matrix6d covariance =
            liorf::uncertainty::Matrix6d::Zero();
        bool valid = false;
    };

    // The estimated transform taking this robot's map frame into `robot_id_th`'s,
    // with the uncertainty recorded when it was optimized.
    MapAlignment mapAlignment(int robot_id_th) const {
        MapAlignment alignment;
        const auto pose_it = _global_map_trans_optimized.find(robot_id_th);
        if (pose_it == _global_map_trans_optimized.end())
            return alignment;

        const PointTypePose & trans = pose_it->second;
        alignment.pose = gtsam::Pose3(
            gtsam::Rot3::RzRyRx(trans.roll, trans.pitch, trans.yaw),
            gtsam::Point3(trans.x, trans.y, trans.z));

        const auto covariance_it =
            _global_map_trans_covariance.find(robot_id_th);
        alignment.covariance = covariance_it == _global_map_trans_covariance.end()
            ? alignmentUncertaintyFloor()
            : covariance_it->second;
        alignment.valid =
            liorf::uncertainty::validCovariance(alignment.covariance);
        return alignment;
    }

    void gtsamExpressionGraph(){
        if (_loop_accept_queue[_robot_this_th].size()<2)
            return;

        gtsam::Pose3 initial_pose_0, initial_pose_1;
        const auto& latest = _pose_queue[_robot_this_th][
            _loop_accept_queue[_robot_this_th].back()];
        initial_pose_0 = latest.source_pose;
        initial_pose_1 = latest.aligned_source_pose;

        gtsam::Values initial;
        gtsam::ExpressionFactorGraph graph;

        gtsam::Pose3_ trans(0);

        initial.insert(0, initial_pose_1 * initial_pose_0.inverse());
        //initial.print();

        for (auto i:_loop_accept_queue[_robot_this_th]){
            const auto& registration = _pose_queue[_robot_this_th][i];

            gtsam::Pose3_ predicted = transformTo(
                trans, registration.aligned_source_pose);

            auto measurementNoise = gtsam::noiseModel::Gaussian::Covariance(
                registration.covariance);

            // Add the Pose3 expression variable, an initial estimate, and the measurement noise.
            graph.addExpressionFactor(
                predicted, registration.source_pose, measurementNoise);
        }
        gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();

        gtsam::Pose3 est = result.at<gtsam::Pose3>(0);

        // The alignment is an estimate, and a cross-peer loop constraint
        // composed through it inherits its error. Recover the marginal and
        // floor it: the estimator does not model drift between the two maps
        // or the correlation between this alignment and the registrations it
        // was fitted to, so it must never look better than the floor.
        liorf::uncertainty::Matrix6d alignment_covariance =
            alignmentUncertaintyFloor();
        try {
            gtsam::Marginals marginals(graph, result);
            const liorf::uncertainty::Matrix6d marginal =
                marginals.marginalCovariance(0);
            if (liorf::uncertainty::validCovariance(marginal))
                alignment_covariance += marginal;
            else
                RCLCPP_WARN(get_logger(),
                    "Map alignment marginal for robot %d is not a valid "
                    "covariance; using the configured floor",
                    _robot_this_th);
        } catch (const std::exception & error) {
            RCLCPP_WARN(get_logger(),
                "Map alignment marginal for robot %d unavailable (%s); "
                "using the configured floor",
                _robot_this_th, error.what());
        }
        _global_map_trans_covariance[_robot_this_th] = alignment_covariance;

        PointTypePose map_trans_this;

        //float x, y, z, roll, pitch, yaw;
        map_trans_this.x = est.translation().x();
        map_trans_this.y = est.translation().y();
        map_trans_this.z = est.translation().z();
        map_trans_this.roll  = est.rotation().roll();
        map_trans_this.pitch = est.rotation().pitch();
        map_trans_this.yaw   = est.rotation().yaw();

        auto ite = _global_map_trans.find(_robot_this_th);
        if(ite == _global_map_trans.end()){
            std::vector<PointTypePose> tmp_pose_list;
            tmp_pose_list.push_back(map_trans_this);
            _global_map_trans.emplace(std::make_pair( _robot_this_th,  tmp_pose_list ) );
            _global_map_trans_optimized.emplace(std::make_pair( _robot_this_th, map_trans_this));
        }
        else{
            _global_map_trans[_robot_this_th].push_back(map_trans_this);
            _global_map_trans_optimized[_robot_this_th] = map_trans_this;
        }


        if (_global_odom_trans.size() != 0)
            gtsamFactorGraph();

        if (!_have_trans_to_publish){
            ite = _global_map_trans.find(0);
            if(ite == _global_map_trans.end())
                return;
            _global_map_trans_optimized[0].intensity = 1;
            _trans_to_publish = _global_map_trans_optimized[0];
            _have_trans_to_publish = true;
        }

        if (_global_map_trans.size() == 1  && _global_odom_trans.size() == 0)
            _trans_to_publish = _global_map_trans_optimized[0];

        graph.resize(0);

    }

    void sendMapOutputMessage(){
        if (!_have_trans_to_publish || _map_fusion_frame.empty() ||
            _map_frame.empty())
            return;

        // Publish the fleet-map -> per-platform-map alignment. Local
        // map -> odom correction is owned by TransformFusion and is never
        // carried on this transport.
        nav_msgs::msg::Odometry odom2map;
        odom2map.header.stamp = _cloud_header.stamp;
        odom2map.header.frame_id = _map_fusion_frame;
        odom2map.child_frame_id = _map_frame;
        odom2map.pose.pose.position.x = _trans_to_publish.x;
        odom2map.pose.pose.position.y = _trans_to_publish.y;
        odom2map.pose.pose.position.z = _trans_to_publish.z;
        odom2map.pose.pose.orientation = createQuaternionMsgFromRollPitchYaw
            (_trans_to_publish.roll, _trans_to_publish.pitch, _trans_to_publish.yaw);
        _pub_trans_odom2map->publish(odom2map);
    }

    void sendGlobalLoopMessageKDTree(){

        auto loop_list = _loop_queue[_robot_this_th];
        int len_loop_list = loop_list.size() - 1;
        auto loop_this = loop_list[len_loop_list];
        int id_bin_this = loop_this.source_bin;

        if (_initial_loop.first == -1){
            //if not enough time distance
            _initial_loop.first = _robot_this_th;
            _initial_loop.second = len_loop_list;
            //initialize last point
            _id_bin_last = id_bin_this;
            return;
        }

        if (!compare_timestamp(id_bin_this, _id_bin_last)){
            //if not enough history point
            return;
        }

        int tmp_robot_id_th =  _initial_loop.first;
        int tmp_len_loop_list = _initial_loop.second;
        auto loop_that = _loop_queue[tmp_robot_id_th][tmp_len_loop_list];
        sendLoopThis(_robot_this_th, tmp_robot_id_th, len_loop_list, tmp_len_loop_list);
        sendLoopThat(_robot_this_th, tmp_robot_id_th, len_loop_list, tmp_len_loop_list);

        _id_bin_last = id_bin_this;

    }


    bool compare_timestamp(int id_0, int id_1){
        if(abs(_bin_with_id[id_0].pose.intensity - _bin_with_id[id_1].pose.intensity) > _loop_frame_thres)
            return true;
        else
            return false;
    }

    void sendLoopThis(int robot_id_this, int robot_id_that, int id_loop_this, int id_loop_that){
        auto loop_list_this = _loop_queue[robot_id_this];
        auto loop_list_that = _loop_queue[robot_id_that];
        auto loop_this = loop_list_this[id_loop_this];
        auto loop_that = loop_list_that[id_loop_that];

        int id_bin_this = loop_this.source_bin;
        int id_bin_last = loop_that.source_bin;

        auto pose_this = _pose_queue[robot_id_this][id_loop_this];
        auto pose_that = _pose_queue[robot_id_that][id_loop_that];

        liorf::uncertainty::PoseWithCovariance pose_to_this;
        liorf::uncertainty::PoseWithCovariance pose_to_that;
        if(robot_id_this == robot_id_that){
            pose_to_this = {pose_this.aligned_source_pose, pose_this.covariance};
            pose_to_that = {pose_that.aligned_source_pose, pose_that.covariance};
        }
        else{
            // The two endpoints were registered against different peers, so
            // each is expressed in that peer's map frame and has to come back
            // through its alignment before they can be differenced. The
            // alignment is estimated, and its uncertainty belongs in the
            // resulting factor.
            //
            // The alignment and the registration are correlated -- the
            // alignment was fitted to registrations including this one -- and
            // this treats them as independent. That errs towards a larger
            // covariance, which is the safe direction; treating the alignment
            // as exact, as this previously did, errs towards a smaller one.
            const MapAlignment alignment_this = mapAlignment(robot_id_this);
            const MapAlignment alignment_that = mapAlignment(robot_id_that);
            if (!alignment_this.valid || !alignment_that.valid)
                return;

            const auto inverse_this = liorf::uncertainty::inverse(
                alignment_this.pose, alignment_this.covariance);
            const auto inverse_that = liorf::uncertainty::inverse(
                alignment_that.pose, alignment_that.covariance);

            pose_to_this = liorf::uncertainty::compose(
                inverse_this.pose, inverse_this.covariance,
                pose_this.aligned_source_pose, pose_this.covariance);
            pose_to_that = liorf::uncertainty::compose(
                inverse_that.pose, inverse_that.covariance,
                pose_that.aligned_source_pose, pose_that.covariance);

            if (!liorf::uncertainty::validCovariance(pose_to_this.covariance) ||
                !liorf::uncertainty::validCovariance(pose_to_that.covariance)) {
                RCLCPP_WARN(get_logger(),
                    "Cross-peer loop factor between robots %d and %d dropped: "
                    "map-alignment propagation produced an invalid covariance",
                    robot_id_that, robot_id_this);
                return;
            }
        }

        const auto measurement = liorf::uncertainty::between(
            pose_to_that.pose, pose_to_that.covariance,
            pose_to_this.pose, pose_to_this.covariance);
        update_loop_info(
            id_bin_last, id_bin_this, measurement,
            0.5 * (pose_this.truncated_mse_m2 + pose_that.truncated_mse_m2),
            std::min(pose_this.overlap_ratio, pose_that.overlap_ratio),
            std::min(pose_this.inliers, pose_that.inliers));

        _pub_loop_info->publish(_loop_info);
    }

    void sendLoopThat(int robot_id_this,int robot_id_that, int id_loop_this, int id_loop_that){
        if(robot_id_this != robot_id_that)
            return;
        auto loop_list = _loop_queue[robot_id_this];
        auto loop_this = loop_list[id_loop_this];
        auto loop_that = loop_list[id_loop_that];

        int id_bin_this = loop_this.target_bin;
        int id_bin_last = loop_that.target_bin;

        auto pose_this = _pose_queue[robot_id_this][id_loop_this];
        auto pose_that = _pose_queue[robot_id_this][id_loop_that];
        const auto pose_to_this = liorf::uncertainty::compose(
            pose_this.source_pose, liorf::uncertainty::Matrix6d::Zero(),
            loop_this.relative_pose, loop_this.covariance);
        const auto pose_to_that = liorf::uncertainty::compose(
            pose_that.source_pose, liorf::uncertainty::Matrix6d::Zero(),
            loop_that.relative_pose, loop_that.covariance);
        const auto measurement = liorf::uncertainty::between(
            pose_to_that.pose, pose_to_that.covariance,
            pose_to_this.pose, pose_to_this.covariance);
        update_loop_info(
            id_bin_last, id_bin_this, measurement,
            0.5 * (pose_this.truncated_mse_m2 + pose_that.truncated_mse_m2),
            std::min(pose_this.overlap_ratio, pose_that.overlap_ratio),
            std::min(pose_this.inliers, pose_that.inliers));

        _pub_loop_info_global->publish(_loop_info);
    }

    void update_loop_info(
            int id_bin_last,
            int id_bin_this,
            const liorf::uncertainty::PoseWithCovariance& measurement,
            double registration_error_m2,
            double overlap_ratio,
            std::size_t registration_inliers){
        _loop_info.header = _cloud_header;
        liorf::loop_constraint::populate(
            _loop_info,
            _bin_with_id[id_bin_last].robotname,
            static_cast<std::int64_t>(_bin_with_id[id_bin_last].pose.intensity),
            static_cast<std::int64_t>(_bin_with_id[id_bin_this].pose.intensity),
            measurement,
            registration_error_m2,
            overlap_ratio,
            registration_inliers);
    }

    void sendOdomOutputMessage(){

        sendMapOutputMessage();

        //publish relative transformation to other robots
        nav_msgs::msg::Odometry odom2odom;
        odom2odom.header.stamp = _cloud_header.stamp;
        odom2odom.header.frame_id = _robot_id;
        odom2odom.child_frame_id = _robot_this;
        odom2odom.pose.pose.position.x = _global_map_trans_optimized[_robot_this_th].x;
        odom2odom.pose.pose.position.y = _global_map_trans_optimized[_robot_this_th].y;
        odom2odom.pose.pose.position.z = _global_map_trans_optimized[_robot_this_th].z;
        odom2odom.pose.pose.orientation = createQuaternionMsgFromRollPitchYaw
            (_global_map_trans_optimized[_robot_this_th].roll,
             _global_map_trans_optimized[_robot_this_th].pitch,
             _global_map_trans_optimized[_robot_this_th].yaw);
        _pub_trans_odom2odom->publish(odom2odom);

        sendGlobalLoopMessageKDTree();

    }
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    auto MapF = std::make_shared<MapFusion>(options);

    RCLCPP_INFO(MapF->get_logger(), "\033[1;32m----> Map Fusion Started.\033[0m");

    std::thread publishThread(&MapFusion::publishContextInfoThread, MapF);

    rclcpp::spin(MapF);

    rclcpp::shutdown();

    publishThread.join();

    return 0;
}
