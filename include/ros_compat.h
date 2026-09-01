#pragma once
#ifndef _LIORF_ROS_COMPAT_H_
#define _LIORF_ROS_COMPAT_H_

// Small shims that replace the ROS 1 conveniences this package relied on
// (ros::Time::toSec(), tf::createQuaternion*, publisher subscriber counts).
// Shared by utility.h and the map fusion nodes.

#include <rclcpp/rclcpp.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// tf2 renamed its headers to .hpp; the .h shims are deprecated and are being
// removed. Prefer .hpp and fall back so older distros still build.
#if __has_include(<tf2/LinearMath/Quaternion.hpp>)
  #include <tf2/LinearMath/Quaternion.hpp>
  #include <tf2/LinearMath/Matrix3x3.hpp>
  #include <tf2/LinearMath/Transform.hpp>
#else
  #include <tf2/LinearMath/Quaternion.h>
  #include <tf2/LinearMath/Matrix3x3.h>
  #include <tf2/LinearMath/Transform.h>
#endif
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <cstdint>
#include <string>

// ROS 2 keeps message stamps as builtin_interfaces/Time; ROS 1's Time::toSec()
// has no direct equivalent, so convert explicitly.
template<typename T>
inline double stamp2Sec(const T& stamp)
{
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

inline rclcpp::Time sec2Stamp(const double seconds)
{
    return rclcpp::Time(static_cast<int64_t>(seconds * 1e9), RCL_ROS_TIME);
}

inline geometry_msgs::msg::Quaternion createQuaternionMsgFromRollPitchYaw(
    const double roll, const double pitch, const double yaw)
{
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return tf2::toMsg(q);
}

inline tf2::Quaternion createQuaternionFromRPY(const double roll, const double pitch, const double yaw)
{
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return q;
}

inline void quaternionMsgToTF2(const geometry_msgs::msg::Quaternion& msg, tf2::Quaternion& q)
{
    tf2::fromMsg(msg, q);
}

// Converts a PCL cloud to a PointCloud2 and publishes it when somebody is
// listening. ``thisPub`` may be null, in which case the message is only built.
template<typename T>
sensor_msgs::msg::PointCloud2 publishCloud(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& thisPub,
    const T& thisCloud, const rclcpp::Time& thisStamp, const std::string& thisFrame)
{
    sensor_msgs::msg::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub && thisPub->get_subscription_count() != 0)
        thisPub->publish(tempCloud);
    return tempCloud;
}

template<typename T>
double ROS_TIME(T msg)
{
    return stamp2Sec(msg->header.stamp);
}

// Joins a robot id with a topic name from the configuration. A topic that is
// already absolute (leading '/') is left alone, as ROS naming rules require;
// blindly concatenating would produce an invalid "robot//topic", which ROS 2
// rejects at subscription time.
inline std::string prefixTopic(const std::string & robot_id, const std::string & topic)
{
    if (topic.empty() || topic.front() == '/')
        return topic;
    if (robot_id.empty())
        return topic;
    return robot_id + "/" + topic;
}

#endif  // _LIORF_ROS_COMPAT_H_
