#pragma once

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_srvs/SetBool.h>

#include <pcl_ros/point_cloud.h>

#include <Eigen/Dense>

#include <vector>
#include <chrono>

using PointCloud = pcl::PointCloud<pcl::PointXYZRGBNormal>;
using PointCloudPtr = std::shared_ptr<PointCloud>;

struct CostMapPoint {
    Eigen::Vector2f point{};
    uint8_t cost{};
    size_t frameNumberSeen{};
};

class CostMapNode {
private:
    ros::NodeHandle mNh;
    ros::NodeHandle mPnh;

    ros::Publisher mCostMapPub;
    ros::NodeHandle mCMt;

    ros::Subscriber mPcSub;
    tf2_ros::Buffer mTfBuffer;
    tf2_ros::TransformListener mTfListener;

    std::vector<CostMapPoint> mCostMapPoints;
    size_t mFrameNumber = 0;

    bool mPublishCostMaps = false;
    bool mIsVerbose = false;

    void pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const &msg);
    std::pair<int, int> convertToCell(float pointx, float pointy);

    nav_msgs::OccupancyGrid mLocalGrid;
    PointCloudPtr mCloudPtr = std::make_shared<PointCloud>();

public:
    CostMapNode();
};