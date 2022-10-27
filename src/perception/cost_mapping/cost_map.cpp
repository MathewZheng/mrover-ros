#include "cost_map.hpp"


// calculation should be here
void CostMapNode::pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg) {

}


CostMapNode::CostMapNode() : mNh(), mPnh("~"), mTfListener(mTfBuffer) {
    mPnh.param<bool>("publish_cost_maps", mPublishCostMaps, true);
    mPnh.param<bool>("verbose", mIsVerbose, false);

    // TODO: need to advertise cost maps

    mPcSub = mNh.subscribe("camera/depth/points", 1, &CostMapNode::pointCloudCallback, this);

    ROS_INFO("Cost map ready");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "cost_map");

    [[maybe_unused]] auto node = std::make_unique<CostMapNode>();

    ros::spin();

    return EXIT_SUCCESS;
}