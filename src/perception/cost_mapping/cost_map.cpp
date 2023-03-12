#include "cost_map.hpp"
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

CostMapNode::CostMapNode() : mNh(), mPnh("~"), mTfListener(mTfBuffer) {
    mPnh.param<bool>("publish_cost_maps", mPublishCostMaps, true);
    mPnh.param<bool>("verbose", mIsVerbose, false);

    mCostMapPub = mCMt.advertise<nav_msgs::OccupancyGrid>("cost_maps", 1);

    mPcSub = mNh.subscribe("camera/depth/points", 1, &CostMapNode::pointCloudCallback, this);

    mCostMapPoints.resize(64, std::vector<std::vector<CostMapPoint>>(64, std::vector<CostMapPoint>(BIN_MAX, {{0, 0}, -1})));
    mCostMapPointsScratch.resize(64, std::vector<std::vector<CostMapPoint>>(64, std::vector<CostMapPoint>(BIN_MAX, {{0, 0}, -1})));

    mLocalGrid.data.resize(4096, -1);
    mLocalGrid.info.width = 64;
    mLocalGrid.info.height = 64;
    mLocalGrid.info.resolution = 1;
    mLocalGrid.info.origin.position.x = -32;
    mLocalGrid.info.origin.position.y = -32;

    ROS_INFO("Cost map ready");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "cost_map");
        
    // ------------------------------------
    // -----Create example point cloud-----
    // ------------------------------------
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    std::cout << "Generating example point clouds.\n\n";
    // We're going to make an ellipse extruded along the z-axis. The colour for
    // the XYZRGB cloud will gradually go from red to green to blue.
    for (float x(-20.0); x <= 20.0; x += 0.05) {
        for (float y(-20.0); y <= 20.0; y += 0.05) {
            pcl::PointXYZRGBNormal point;
            point.x = x;
            point.y = y;
            point.z = 0;
            // point.r = 255;
            // point.g = 255;
            // point.b = 255;
            
            point_cloud_ptr->points.push_back(point);
        }
    }
    point_cloud_ptr->width = point_cloud_ptr->size();
    point_cloud_ptr->height = 1;

    [[maybe_unused]] auto node = std::make_unique<CostMapNode>();

    sensor_msgs::PointCloud2Ptr msg = boost::make_shared<sensor_msgs::PointCloud2>();
    pcl::toROSMsg(*point_cloud_ptr, *msg);
    // node->pointCloudCallback(msg);
    while (ros::ok()) {
        ros::spinOnce();
        // node->pointCloudCallback(msg);
    }

    // ros::spin();

    return EXIT_SUCCESS;
}