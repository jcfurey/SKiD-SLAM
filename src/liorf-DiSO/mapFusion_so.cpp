//
// Created by yewei on 8/31/20.
// Modified by Hogyun Kim on 11/07/24

//

//msg
#include "liorf/msg/cloud_info.hpp"
#include "liorf/msg/context_info.hpp"

//third party
#include "SOLiD/solid.h"
#include "fast_max-clique_finder/src/findClique.h"

#include "nabo/nabo.h"
#include "skid_registration.hpp"

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
    rclcpp::Subscription<liorf::msg::ContextInfo>::SharedPtr _sub_loop_info_global;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_communication_signal;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_signal_1;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_signal_2;

    rclcpp::CallbackGroup::SharedPtr _callback_group;


    std::string _signal_id_1;
    std::string _signal_id_2;

    rclcpp::Publisher<liorf::msg::ContextInfo>::SharedPtr _pub_context_info;
    rclcpp::Publisher<liorf::msg::ContextInfo>::SharedPtr _pub_loop_info;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_cloud;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _pub_trans_odom2map;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _pub_trans_odom2odom;

    rclcpp::Publisher<liorf::msg::ContextInfo>::SharedPtr _pub_loop_info_global;

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

    liorf::registration::Config _registration_config;

    std::mutex mtx_publish_1;
    std::mutex mtx_publish_2;
    std::mutex mtx;

    std::string _robot_initial;
    std::string _pcm_matrix_folder;

    liorf::msg::CloudInfo   _cloud_info;
//    liorf::msg::ContextInfo _context_info;
    std::vector<SOLiDBin> _context_list_to_publish_1;
    std::vector<SOLiDBin> _context_list_to_publish_2;

    pcl::KdTreeFLANN<PointType>::Ptr _kdtree_pose_to_publish;
    pcl::PointCloud<PointType>::Ptr _cloud_pose_to_publish;

    pcl::KdTreeFLANN<PointType>::Ptr _kdtree_pose_to_search;
    pcl::PointCloud<PointType>::Ptr _cloud_pose_to_search_this;//3D
    pcl::PointCloud<PointType>::Ptr _cloud_pose_to_search_other;//3D

    pcl::KdTreeFLANN<PointType>::Ptr _kdtree_loop_to_search;
    pcl::PointCloud<PointType>::Ptr _cloud_loop_to_search;

    std::pair<int, int> _initial_loop;
    int _id_bin_last;

    liorf::msg::ContextInfo _loop_info;
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

    //for pcm & graph
    //first: source pose; second: source pose in target; third: truncated MSE;
    std::unordered_map< int, std::vector< std::tuple<gtsam::Pose3, gtsam::Pose3, float> > >  _pose_queue;
    //first: target, second: source, third: relative transform;
    std::unordered_map< int, std::vector< std::tuple<int, int, gtsam::Pose3> > > _loop_queue;

    std::unordered_map< std::string, std::vector<PointTypePose> > _global_odom_trans;

    //first: robot pair id, second: effective loop id in _loop_queue
    std::unordered_map< int, std::vector<int> > _loop_accept_queue;

    std::unordered_map< int, std::vector<PointTypePose> > _global_map_trans;
    std::unordered_map< int, PointTypePose> _global_map_trans_optimized;

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

        _sub_loop_info_global = create_subscription<liorf::msg::ContextInfo>(
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



        _pub_context_info = create_publisher<liorf::msg::ContextInfo>(_solid_topic + "/context_info", 1);
        _pub_loop_info = create_publisher<liorf::msg::ContextInfo>(
                prefixTopic(_robot_id, _loop_topic), 1);
        _pub_cloud = create_publisher<sensor_msgs::msg::PointCloud2>(_robot_id + "/" + _solid_topic + "/cloud", 1);
        _pub_trans_odom2map = create_publisher<nav_msgs::msg::Odometry>(_robot_id + "/" + _solid_topic + "/trans_map", 1);
        _pub_trans_odom2odom = create_publisher<nav_msgs::msg::Odometry>(_solid_topic + "/trans_odom", 1);
        _pub_loop_info_global = create_publisher<liorf::msg::ContextInfo>(_solid_topic + "/loop_info_global", 1);

    }

    void publishContextInfoThread(){
        int signal_id_th_1 = _signal_id_1.empty() ? -1 : robotID2Number(_signal_id_1);
        int signal_id_th_2 = _signal_id_2.empty() ? -1 : robotID2Number(_signal_id_2);
        while (rclcpp::ok())
        {
            if (!_signal_id_1.empty() && _communication_signal && _signal_1 &&
                _robot_id_th < signal_id_th_1){
                if (_context_list_to_publish_1.empty())
                    continue;
                //publish solid info to other robots
                mtx_publish_1.lock();
                SOLiDBin bin = _context_list_to_publish_1.back();
                _context_list_to_publish_1.pop_back();
                mtx_publish_1.unlock();
                publishContextInfo(bin, _signal_id_1);
            }
            if (!_signal_id_2.empty() && _communication_signal && _signal_2 &&
                _robot_id_th < signal_id_th_2){
                if (_context_list_to_publish_2.empty())
                    continue;
                //publish solid info to other robots
                mtx_publish_2.lock();
                SOLiDBin bin = _context_list_to_publish_2.back();
                _context_list_to_publish_2.pop_back();
                mtx_publish_2.unlock();
                publishContextInfo(bin, _signal_id_2);

            }
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
        _pcm_thres = declare_and_get<double>("mapfusion.interRobot.pcm_threshold", 20.0);
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

        _registration_config.coarse_voxel_size_m = declare_and_get<double>(
            "mapfusion.registration.coarse_voxel_size_m", 2.0);
        _registration_config.coarse_use_voxel_sampling = declare_and_get<bool>(
            "mapfusion.registration.coarse_use_voxel_sampling", true);
        _registration_config.coarse_use_quatro = declare_and_get<bool>(
            "mapfusion.registration.coarse_use_quatro", false);
        _registration_config.coarse_linearity_threshold = declare_and_get<double>(
            "mapfusion.registration.coarse_linearity_threshold", 0.99);
        _registration_config.coarse_max_correspondences = declare_and_get<int>(
            "mapfusion.registration.coarse_max_correspondences", 5000);
        _registration_config.coarse_normal_radius_gain = declare_and_get<double>(
            "mapfusion.registration.coarse_normal_radius_gain", 3.5);
        _registration_config.coarse_fpfh_radius_gain = declare_and_get<double>(
            "mapfusion.registration.coarse_fpfh_radius_gain", 5.0);
        _registration_config.coarse_robin_noise_bound_gain = declare_and_get<double>(
            "mapfusion.registration.coarse_robin_noise_bound_gain", 1.0);
        _registration_config.coarse_solver_noise_bound_gain = declare_and_get<double>(
            "mapfusion.registration.coarse_solver_noise_bound_gain", 0.75);
        _registration_config.coarse_clamp_noise_bounds = declare_and_get<bool>(
            "mapfusion.registration.coarse_clamp_noise_bounds", true);

        const int min_coarse_correspondences = declare_and_get<int>(
            "mapfusion.registration.min_coarse_correspondences", 5);
        const int min_coarse_inliers = declare_and_get<int>(
            "mapfusion.registration.min_coarse_inliers", 3);
        const int min_fine_inliers = declare_and_get<int>(
            "mapfusion.registration.min_fine_inliers", 20);
        const int min_metric_inliers = declare_and_get<int>(
            "mapfusion.registration.min_metric_inliers", 20);
        if (min_coarse_correspondences < 0 || min_coarse_inliers < 0 ||
            min_fine_inliers < 0 || min_metric_inliers < 0) {
            throw std::invalid_argument(
                "mapfusion registration inlier counts cannot be negative");
        }
        _registration_config.min_coarse_correspondences =
            static_cast<std::size_t>(min_coarse_correspondences);
        _registration_config.min_coarse_inliers =
            static_cast<std::size_t>(min_coarse_inliers);
        _registration_config.min_fine_inliers =
            static_cast<std::size_t>(min_fine_inliers);
        _registration_config.min_metric_inliers =
            static_cast<std::size_t>(min_metric_inliers);

        _registration_config.fine_downsampling_resolution_m = declare_and_get<double>(
            "mapfusion.registration.fine_downsampling_resolution_m", 0.5);
        _registration_config.fine_max_correspondence_distance_m = declare_and_get<double>(
            "mapfusion.registration.fine_max_correspondence_distance_m", 2.0);
        _registration_config.fine_rotation_epsilon_rad = declare_and_get<double>(
            "mapfusion.registration.fine_rotation_epsilon_rad", 0.0017453292519943296);
        _registration_config.fine_translation_epsilon_m = declare_and_get<double>(
            "mapfusion.registration.fine_translation_epsilon_m", 0.001);
        _registration_config.fine_num_neighbors = declare_and_get<int>(
            "mapfusion.registration.fine_num_neighbors", 10);
        _registration_config.fine_num_threads = declare_and_get<int>(
            "mapfusion.registration.fine_num_threads", 4);
        _registration_config.fine_max_iterations = declare_and_get<int>(
            "mapfusion.registration.fine_max_iterations", 30);

        _registration_config.truncated_mse_max_correspondence_distance_m =
            declare_and_get<double>(
                "mapfusion.registration.truncated_mse_max_correspondence_distance_m", 3.0);
        _registration_config.max_truncated_mse_m2 = declare_and_get<double>(
            "mapfusion.registration.max_truncated_mse_m2", legacy_icp_threshold);
        _registration_config.min_overlap_ratio = declare_and_get<double>(
            "mapfusion.registration.min_overlap_ratio", 0.10);

        const std::string registration_error =
            liorf::registration::validate(_registration_config);
        if (!registration_error.empty()) {
            throw std::invalid_argument(
                "invalid mapfusion.registration configuration: " + registration_error);
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

        _trans_to_publish.intensity = 0;

        _num_bin = 0;//

        _communication_signal = true;
        _signal_1 = true;
        _signal_2 = true;


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

        bin.cloud->clear();
        pcl::copyPointCloud(*_laser_cloud_feature,  *bin.cloud);

        //push the bin info into the wait list
        mtx_publish_1.lock();
        _context_list_to_publish_1.push_back(bin);
        mtx_publish_1.unlock();
        mtx_publish_2.lock();
        _context_list_to_publish_2.push_back(bin);
        mtx_publish_2.unlock();


        publishContextInfo(bin, _robot_id);

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
        context_info.pose_x = bin.pose.x;
        context_info.pose_y = bin.pose.y;
        context_info.pose_z = bin.pose.z;
        context_info.pose_roll  =  bin.pose.roll;
        context_info.pose_pitch =  bin.pose.pitch;
        context_info.pose_yaw   =  bin.pose.yaw;
        context_info.pose_intensity = bin.pose.intensity;

        context_info.scan_cloud =  publishCloud(_pub_cloud, bin.cloud, sec2Stamp(bin.time), _robot_id + "/" + _solid_frame);
        mtx.lock();
        _pub_context_info->publish(context_info);
        mtx.unlock();
//        context_info.SOLiDBin.
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

    void globalLoopInfoHandler(const liorf::msg::ContextInfo::ConstSharedPtr& msgIn){
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
        bin.pose.intensity = msgIn->pose_intensity;

        bin.cloud.reset(new pcl::PointCloud<PointType>());
        pcl::fromROSMsg(msgIn->scan_cloud, *bin.cloud);

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

        if(!getInitialGuesses(bin)){
            return;
        }

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
        //store data received
        _bin_with_id.emplace( std::make_pair(_num_bin-1, bin) );

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

    bool getInitialGuesses(SOLiDBin bin){
        if(_idx_nearest_list.size() == 0){
            return false;
        }
        bool new_candidate_signal = false;
        for (auto it: _idx_nearest_list){
            new_candidate_signal = getInitialGuess(bin, it.first, it.second);
        }
        return new_candidate_signal;
    }

    bool getInitialGuess(SOLiDBin bin, int idx_nearest, int min_idx){

        int id0 = idx_nearest, id1 = _num_bin - 1;

        SOLiDBin bin_nearest;
        PointTypePose source_pose_initial, target_pose;

        float solid_pitch = (min_idx+1) * M_PI * 2 /_num_sectors;
        if (solid_pitch > M_PI)
            solid_pitch -= (M_PI * 2);

        int robot_id_this = robotID2Number(bin.robotname);

        auto robot_id_this_ite = std::find(_robot_received_list.begin(),  _robot_received_list.end(), robot_id_this);

        //record all received robot id from other robots
        if (robot_id_this_ite == _robot_received_list.end() && robot_id_this != _robot_id_th)
            _robot_received_list.push_back(robot_id_this);

        //  exchange if source has a prior robot id (the last character of the robot name is smaller) (first > second)
        if (robot_id_this < _robot_id_th){
            bin_nearest = bin;
            bin = _bin_with_id.at(idx_nearest);

            id0 = _num_bin - 1;
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

        PointTypePose pose_source_lidar = registerRelativeMotion(
            transformPointCloud(bin.cloud, &source_pose_initial),
            transformPointCloud(bin_nearest.cloud, &target_pose),
            source_pose_initial);

        if (pose_source_lidar.intensity < 0.0F)
            return false;

        //1: jackal0, 2: jackal1
        gtsam::Pose3 pose_from =
            gtsam::Pose3(gtsam::Rot3::RzRyRx(bin.pose.roll, bin.pose.pitch, bin.pose.yaw),
                  gtsam::Point3(bin.pose.x, bin.pose.y, bin.pose.z));

        gtsam::Pose3 pose_to =
            gtsam::Pose3(gtsam::Rot3::RzRyRx(pose_source_lidar.roll, pose_source_lidar.pitch, pose_source_lidar.yaw),
                  gtsam::Point3(pose_source_lidar.x, pose_source_lidar.y, pose_source_lidar.z));

        gtsam::Pose3 pose_target =
            gtsam::Pose3(gtsam::Rot3::RzRyRx(target_pose.roll, target_pose.pitch, target_pose.yaw),
                  gtsam::Point3(target_pose.x, target_pose.y, target_pose.z));

        auto ite = _pose_queue.find(_robot_this_th);
        if(ite == _pose_queue.end()){
            std::vector< std::tuple<gtsam::Pose3, gtsam::Pose3, float> > new_pose_queue;
            std::vector< std::tuple<int, int, gtsam::Pose3> > new_loop_queue;
            _pose_queue.emplace( std::make_pair(_robot_this_th, new_pose_queue) );
            _loop_queue.emplace( std::make_pair(_robot_this_th, new_loop_queue) );
        }


        _pose_queue[_robot_this_th].emplace_back(std::make_tuple(pose_from, pose_to, pose_source_lidar.intensity));
        _loop_queue[_robot_this_th].emplace_back(std::make_tuple(id0, id1, pose_to.between(pose_target)));

        return true;
    }

    float distBtnSOLiDs(Eigen::VectorXf rsolid1, Eigen::VectorXf rsolid2, Eigen::VectorXf asolid1, Eigen::VectorXf asolid2, int & idx){
        // calculate min cosine distance of R-SOLiD
        float sim = (rsolid1.dot(rsolid2))/(rsolid1.norm() * rsolid2.norm());
        float dist = 1-sim;

        float minL1normDist = std::numeric_limits<float>::max();
        int minIndex = 0;
        int numAngle = asolid1.size();

        for(int shiftIndex = 0; shiftIndex < numAngle; ++shiftIndex)
        {
            Eigen::VectorXf shiftedQuery = Eigen::VectorXf::Zero(numAngle);
            for (int i = 0; i < numAngle; ++i)
            {
                shiftedQuery((i+shiftIndex) % numAngle) = asolid1(i);
                
            }
            float L1NormDist = (asolid2 - shiftedQuery).cwiseAbs().sum();

            if(L1NormDist < minL1normDist)
            {
                minL1normDist = L1NormDist;
                minIndex = shiftIndex;
            }
        }

        idx = minIndex;

        return dist;
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

    PointTypePose registerRelativeMotion(pcl::PointCloud<PointType>::Ptr source,
                                         pcl::PointCloud<PointType>::Ptr target,
                                         PointTypePose pose_source)
    {
        PointTypePose pose_from;
        pose_from.intensity = -1.0F;

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
            pose_from.intensity = -1;
            return pose_from;
        }

        const Eigen::Affine3f correction_target_from_source(
            registration.T_target_source.matrix().cast<float>());
        const Eigen::Affine3f initial_world_from_lidar = pcl::getTransformation(
            pose_source.x, pose_source.y, pose_source.z,
            pose_source.roll, pose_source.pitch, pose_source.yaw);
        const Eigen::Affine3f corrected_world_from_lidar =
            correction_target_from_source * initial_world_from_lidar;

        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(
            corrected_world_from_lidar, x, y, z, roll, pitch, yaw);

        pose_from.x = x;
        pose_from.y = y;
        pose_from.z = z;

        pose_from.yaw = yaw;
        pose_from.roll = roll;
        pose_from.pitch = pitch;

        pose_from.intensity = static_cast<float>(registration.metric.value_m2);

        RCLCPP_INFO(
            get_logger(),
            "SKiD registration accepted: corr=%zu coarse_inliers=%zu "
            "fine_inliers=%zu overlap=%.3f tMSE=%.6f m^2 "
            "time=%.1f ms (coarse %.1f, fine %.1f)",
            registration.coarse_correspondences,
            registration.coarse_translation_inliers,
            registration.fine_inliers,
            registration.metric.overlap_ratio,
            registration.metric.value_m2,
            1000.0 * (registration.coarse_seconds + registration.fine_seconds +
                      registration.metric_seconds),
            1000.0 * registration.coarse_seconds,
            1000.0 * registration.fine_seconds);

        return pose_from;

    }

    bool incrementalPCM() {
        if (_pose_queue[_robot_this_th].size() < _pcm_start_threshold)
            return false;

        //perform pcm for all robot matches

        Eigen::MatrixXi consistency_matrix = computePCMMatrix(_loop_queue[_robot_this_th]);//, _pose_queue[_robot_this_th]);
        std::string consistency_matrix_file = _pcm_matrix_folder + "/consistency_matrix" + _robot_id + ".clq.mtx";
        printPCMGraph(consistency_matrix, consistency_matrix_file);
        // Compute maximum clique
        FMC::CGraphIO gio;
        gio.readGraph(consistency_matrix_file);
        int max_clique_size = 0;
        std::vector<int> max_clique_data;

        max_clique_size = FMC::maxCliqueHeu(gio, max_clique_data);

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

    Eigen::MatrixXi computePCMMatrix( std::vector< std::tuple<int, int, gtsam::Pose3> > loop_queue_this){
        Eigen::MatrixXi PCMMat;
        PCMMat.setZero(loop_queue_this.size(), loop_queue_this.size());
        int id_0, id_1;
        gtsam::Pose3 z_aj_bk, z_ai_bl;
        gtsam::Pose3 z_ai_aj, z_bk_bl;
        gtsam::Pose3 t_ai, t_aj, t_bk, t_bl;

        for (unsigned int i = 0; i < loop_queue_this.size(); i++){
            std::tie(id_0, id_1, z_aj_bk) = loop_queue_this[i];
            PointTypePose tmp_pose_0 = _bin_with_id.at(id_0).pose;
            PointTypePose tmp_pose_1 = _bin_with_id.at(id_1).pose;
            t_aj = gtsam::Pose3(gtsam::Rot3::RzRyRx(tmp_pose_0.roll, tmp_pose_0.pitch, tmp_pose_0.yaw),
                                gtsam::Point3(tmp_pose_0.x, tmp_pose_0.y, tmp_pose_0.z));
            t_bk = gtsam::Pose3(gtsam::Rot3::RzRyRx(tmp_pose_1.roll, tmp_pose_1.pitch, tmp_pose_1.yaw),
                                gtsam::Point3(tmp_pose_1.x, tmp_pose_1.y, tmp_pose_1.z));

            for (unsigned int j = i + 1; j < loop_queue_this.size(); j++){
                std::tie(id_0, id_1, z_ai_bl) = loop_queue_this[j];
                PointTypePose tmp_pose_0 = _bin_with_id.at(id_0).pose;
                PointTypePose tmp_pose_1 = _bin_with_id.at(id_1).pose;
                t_ai = gtsam::Pose3(gtsam::Rot3::RzRyRx(tmp_pose_0.roll, tmp_pose_0.pitch, tmp_pose_0.yaw),
                                    gtsam::Point3(tmp_pose_0.x, tmp_pose_0.y, tmp_pose_0.z));
                t_bl = gtsam::Pose3(gtsam::Rot3::RzRyRx(tmp_pose_1.roll, tmp_pose_1.pitch, tmp_pose_1.yaw),
                                    gtsam::Point3(tmp_pose_1.x, tmp_pose_1.y, tmp_pose_1.z));
                z_ai_aj = t_ai.between(t_aj);
                z_bk_bl = t_bk.between(t_bl);
                float resi = residualPCM(z_aj_bk, z_ai_bl, z_ai_aj, z_bk_bl, 1);
                if (resi < _pcm_thres)
                    PCMMat(i,j) = 1;
                else
                    PCMMat(i,j) = 0;
            }
        }
        return PCMMat;
    }

    float residualPCM(gtsam::Pose3 inter_jk, gtsam::Pose3 inter_il, gtsam::Pose3 inner_ij, gtsam::Pose3 inner_kl, float intensity){
        gtsam::Pose3 inter_il_inv = inter_il.inverse();
        gtsam::Pose3 res_pose = inner_ij * inter_jk * inner_kl * inter_il_inv;
        gtsam::Vector6 res_vec = gtsam::Pose3::Logmap(res_pose);

        Eigen::Matrix< double, 6, 1> v ;
        v << intensity, intensity, intensity,
            intensity, intensity, intensity;
        Eigen::Matrix< double, 6, 6> m_cov = v.array().matrix().asDiagonal();

        return sqrt(res_vec.transpose()* m_cov * res_vec);
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

    void gtsamExpressionGraph(){
        if (_loop_accept_queue[_robot_this_th].size()<2)
            return;

        gtsam::Vector Vector6(6);

        gtsam::Pose3 initial_pose_0, initial_pose_1;
        initial_pose_0 = std::get<0>(_pose_queue[_robot_this_th][ _loop_accept_queue[_robot_this_th][_loop_accept_queue[_robot_this_th].size() -1] ]);
        initial_pose_1 = std::get<1>(_pose_queue[_robot_this_th][ _loop_accept_queue[_robot_this_th][_loop_accept_queue[_robot_this_th].size() -1] ]);

        gtsam::Values initial;
        gtsam::ExpressionFactorGraph graph;

        gtsam::Pose3_ trans(0);

        initial.insert(0, initial_pose_1 * initial_pose_0.inverse());
        //initial.print();

        gtsam::Pose3 p, measurement;
        float noiseScore;

        for (auto i:_loop_accept_queue[_robot_this_th]){
            std::tie(measurement, p, noiseScore) = _pose_queue[_robot_this_th][i];

            if(noiseScore == 0)// 0 indicates a inter robot outlier
                continue;

            gtsam::Pose3_ predicted = transformTo(trans,p);

            Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore,
                noiseScore;
            auto measurementNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);

            // Add the Pose3 expression variable, an initial estimate, and the measurement noise.
            graph.addExpressionFactor(predicted, measurement, measurementNoise);
        }
        gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();

        gtsam::Pose3 est = result.at<gtsam::Pose3>(0);

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
        int id_bin_this = std::get<1>(loop_this);

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
        int id_bin_that = std::get<1>(loop_that);
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

        int id_bin_this = std::get<1>(loop_this);
        int id_bin_last = std::get<1>(loop_that);

        auto pose_this = _pose_queue[robot_id_this][id_loop_this];
        auto pose_that = _pose_queue[robot_id_that][id_loop_that];

        gtsam::Pose3 pose_to_this, pose_to_that;
        if(robot_id_this == robot_id_that){
            pose_to_this = std::get<1>(pose_this);
            pose_to_that = std::get<1>(pose_that);
        }
        else{
            if (_global_map_trans_optimized.find(robot_id_this) == _global_map_trans_optimized.end() ||
                    _global_map_trans_optimized.find(robot_id_that) == _global_map_trans_optimized.end() )
                return;
            PointTypePose trans_this = _global_map_trans_optimized[robot_id_this];
            gtsam::Pose3 trans_this3 = gtsam::Pose3( gtsam::Rot3::RzRyRx(trans_this.roll, trans_this.pitch, trans_this.yaw),
                                                    gtsam::Point3(trans_this.x, trans_this.y, trans_this.z) );

            PointTypePose trans_that = _global_map_trans_optimized[robot_id_that];
            gtsam::Pose3 trans_that3 = gtsam::Pose3( gtsam::Rot3::RzRyRx(trans_that.roll, trans_that.pitch, trans_that.yaw),
                                                     gtsam::Point3(trans_that.x, trans_that.y, trans_that.z) );

            pose_to_this =  trans_this3.inverse() * std::get<1>(pose_this);
            pose_to_that =  trans_that3.inverse() * std::get<1>(pose_that);
        }

        float tmp_intensity = (std::get<2>(pose_this) + std::get<2>(pose_that) ) / 2.0;

        auto dpose = pose_to_that.between(pose_to_this);
        auto thisp = _bin_with_id[id_bin_this].pose;
        auto thatp = _bin_with_id[id_bin_last].pose;
        auto pose_to_this1 = gtsam::Pose3( gtsam::Rot3::RzRyRx(thisp.roll, thisp.pitch, thisp.yaw),
                                     gtsam::Point3(thisp.x, thisp.y, thisp.z) );
        auto pose_to_that1 = gtsam::Pose3( gtsam::Rot3::RzRyRx(thatp.roll, thatp.pitch, thatp.yaw),
                                     gtsam::Point3(thatp.x, thatp.y, thatp.z) );
        auto dpose1 = pose_to_that1.between(pose_to_this1);

        update_loop_info(id_bin_last, id_bin_this, pose_to_that, pose_to_this, tmp_intensity);

        _pub_loop_info->publish(_loop_info);
    }

    void sendLoopThat(int robot_id_this,int robot_id_that, int id_loop_this, int id_loop_that){
        if(robot_id_this != robot_id_that)
            return;
        auto loop_list = _loop_queue[robot_id_this];
        auto loop_this = loop_list[id_loop_this];
        auto loop_that = loop_list[id_loop_that];

        int id_bin_this = std::get<0>(loop_this);
        int id_bin_last = std::get<0>(loop_that);

        auto pose_this = _pose_queue[robot_id_this][id_loop_this];
        auto pose_that = _pose_queue[robot_id_this][id_loop_that];
        auto pose_to_this = std::get<0>(pose_this) * std::get<2>(loop_this) ;
        auto pose_to_that = std::get<0>(pose_that) * std::get<2>(loop_that) ;
        float tmp_intensity = (std::get<2>(pose_this) + std::get<2>(pose_that) ) / 2.0;
        update_loop_info(id_bin_last, id_bin_this, pose_to_that, pose_to_this, tmp_intensity);

        _pub_loop_info_global->publish(_loop_info);
    }

    void update_loop_info(int id_bin_last, int id_bin_this, gtsam::Pose3 pose_to_last, gtsam::Pose3 pose_to_this, float intensity){
        //relative translation btwn 2 poses
        auto dpose = pose_to_last.between(pose_to_this);
        _loop_info.robot_id = _bin_with_id[id_bin_last].robotname;
        _loop_info.num_ring   = _bin_with_id[id_bin_last].pose.intensity;//from
        _loop_info.num_sector = _bin_with_id[id_bin_this].pose.intensity;//to

        _loop_info.pose_x = dpose.translation().x();
        _loop_info.pose_y = dpose.translation().y();
        _loop_info.pose_z = dpose.translation().z();
        _loop_info.pose_roll  = dpose.rotation().roll();
        _loop_info.pose_pitch = dpose.rotation().pitch();
        _loop_info.pose_yaw   = dpose.rotation().yaw();
        _loop_info.pose_intensity = intensity;

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
