#include "cost_map.hpp"

CostMapNode::CostMapNode() : mNh(), mPnh("~"), mTfListener(mTfBuffer) {
    mPnh.param<bool>("publish_cost_maps", mPublishCostMaps, true);
    mPnh.param<bool>("verbose", mIsVerbose, false);

    mCostMapPub = mCMt.advertise<nav_msgs::OccupancyGrid>("cost_maps", 1);

    mPcSub = mNh.subscribe("camera/depth/points", 1, &CostMapNode::pointCloudCallback, this);
    mPreviousPose = SE3::fromTfTree(mTfBuffer, "map", "base_link");

    ROS_INFO("Cost map ready");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "cost_map");

    [[maybe_unused]] auto node = std::make_unique<CostMapNode>();

    ros::spin();

    return EXIT_SUCCESS;
}