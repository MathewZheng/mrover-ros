#include "cost_map.hpp"
#include <pcl/conversions.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

CostMapNode::CostMapNode() : mNh(), mPnh("~"), mTfListener(mTfBuffer) {
    mPnh.param<bool>("publish_cost_maps", mPublishCostMaps, true);
    mPnh.param<bool>("verbose", mIsVerbose, false);

    mCostMapPub = mCMt.advertise<nav_msgs::OccupancyGrid>("cost_maps", 1);

    mPcSub = mNh.subscribe("camera/depth/points", 1, &CostMapNode::pointCloudCallback, this);

    mCostMapPoints.resize(64, std::vector<std::optional<CostMapPoint>>(64, std::nullopt));
    mCostMapPointsScratch.resize(64, std::vector<std::optional<CostMapPoint>>(64, std::nullopt));

    mLocalGrid.data.resize(4096, -1);
    mLocalGrid.info.width = 64;
    mLocalGrid.info.height = 64;
    mLocalGrid.info.resolution = 1;

    ROS_INFO("Cost map ready");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "cost_map");

    // ------------------------------------
    // -----Create example point cloud-----
    // ------------------------------------
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    std::cout << "Generating example point clouds.\n\n";
    // We're going to make an ellipse extruded along the z-axis. The colour for
    // the XYZRGB cloud will gradually go from red to green to blue.
    std::uint8_t r(255), g(15), b(15);
    for (float z(-20.0); z <= 20.0; z += 0.05)
    {
        for (float angle(0.0); angle <= 360.0; angle += 5.0)
        {
        pcl::PointXYZRGBNormal point;
        point.x = 64 * std::cos (pcl::deg2rad(angle));
        point.y = sinf (pcl::deg2rad(angle));
        point.z = z;
        point.normal_x = 0;
        point.normal_y = 1;
        point.normal_z = 0;
        point_cloud_ptr->points.push_back(point);

        std::uint32_t rgb = (static_cast<std::uint32_t>(r) << 16 |
                static_cast<std::uint32_t>(g) << 8 | static_cast<std::uint32_t>(b));
        point.rgb = *reinterpret_cast<float*>(&rgb);
        point_cloud_ptr->points.push_back (point);
        }
        if (z < 0.0)
        {
        r -= 12;
        g += 12;
        }
        else
        {
        g -= 12;
        b += 12;
        }
    }
    point_cloud_ptr->width = point_cloud_ptr->size ();
    point_cloud_ptr->height = 1;


    [[maybe_unused]] auto node = std::make_unique<CostMapNode>();

    sensor_msgs::PointCloud2Ptr msg = boost::make_shared<sensor_msgs::PointCloud2>();
    pcl::toROSMsg(*point_cloud_ptr, *msg);
    while (ros::ok()) {
        node->pointCloudCallback(msg);
        ros::spinOnce();
    }

    // ros::spin();

    return EXIT_SUCCESS;
}