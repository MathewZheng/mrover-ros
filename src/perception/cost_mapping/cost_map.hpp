#pragma once

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_srvs/SetBool.h>
// #include <buffer.h>
#include <se3.hpp>

#include <pcl_ros/point_cloud.h>

#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>

#include <vector>
#include <chrono>
#include <optional>

using PointCloud = pcl::PointCloud<pcl::PointXYZRGBNormal>;
using PointCloudPtr = std::shared_ptr<PointCloud>;

struct CostMapPoint {
    Eigen::Vector2f point{};
    int8_t cost{};
};

using CostMapGrid = std::vector<std::vector<std::optional<CostMapPoint>>>;

constexpr int8_t MAX_COST = 100;

class CostMapNode {
private:
    ros::NodeHandle mNh;
    ros::NodeHandle mPnh;

    ros::Publisher mCostMapPub;
    ros::NodeHandle mCMt;

    ros::Subscriber mPcSub;
    tf2_ros::Buffer mTfBuffer;
    tf2_ros::TransformListener mTfListener;

    CostMapGrid mCostMapPoints;
    CostMapGrid mCostMapPointsScratch;
    size_t mFrameNumber = 0;

    SE3 mPreviousPose;

    bool mPublishCostMaps = false;
    bool mIsVerbose = false;

    void pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const &msg);

    std::optional<std::pair<size_t, size_t>> convertToCell(Eigen::Vector2f const &point);

    nav_msgs::OccupancyGrid mLocalGrid;
    PointCloudPtr mCloudPtr = std::make_shared<PointCloud>();

public:
    CostMapNode();
};