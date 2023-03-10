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

    // mPcSub = mNh.subscribe("camera/depth/points", 1, &CostMapNode::pointCloudCallback, this);

    mCostMapPoints.resize(64, std::vector<std::optional<CostMapPoint>>(64, std::nullopt));
    mCostMapPointsScratch.resize(64, std::vector<std::optional<CostMapPoint>>(64, std::nullopt));

    mLocalGrid.data.resize(4096, -1);
    mLocalGrid.info.width = 64;
    mLocalGrid.info.height = 64;
    mLocalGrid.info.resolution = 1;

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
    std::uint8_t r(255), g(15), b(15);
    for (float z(-20.0); z <= 20.0; z += 0.05) {
        for (float angle(0.0); angle <= 360.0; angle += 5.0) {
            pcl::PointXYZRGBNormal point;
            point.x = z * std::cos(pcl::deg2rad(angle));
            point.y = z * sinf(pcl::deg2rad(angle));
            point.z = z;
            
            point_cloud_ptr->points.push_back(point);

            std::uint32_t rgb = (static_cast<std::uint32_t>(r) << 16 |
                                 static_cast<std::uint32_t>(g) << 8 | static_cast<std::uint32_t>(b));
            point.rgb = *reinterpret_cast<float*>(&rgb);
            point_cloud_ptr->points.push_back(point);
        }
        if (z < 0.0) {
            r -= 12;
            g += 12;
        } else {
            g -= 12;
            b += 12;
        }
    }
    point_cloud_ptr->width = point_cloud_ptr->size();
    point_cloud_ptr->height = 1;
    pcl::NormalEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> ne;
    ne.setInputCloud(point_cloud_ptr);
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal> ());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    ne.setRadiusSearch(0.03);
    ne.compute(*cloud_normals);


    [[maybe_unused]] auto node = std::make_unique<CostMapNode>();

    sensor_msgs::PointCloud2Ptr msg = boost::make_shared<sensor_msgs::PointCloud2>();
    pcl::toROSMsg(*cloud_normals, *msg);
    node->pointCloudCallback(msg);
    while (ros::ok()) {
        ros::spinOnce();
        node->pointCloudCallback(boost::make_shared<sensor_msgs::PointCloud2>());
    }

    // ros::spin();

    return EXIT_SUCCESS;
}