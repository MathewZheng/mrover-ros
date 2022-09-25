

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/point_cloud2_iterator.h>

class CostMapNode {
private:
    ros::NodeHandle mNh;
    ros::NodeHandle mPnh;

    ros::ServiceServer mServiceEnableCostMaps;

    ros::Subscriber mPcSub;
    tf2_ros::Buffer mTfBuffer;
    tf2_ros::TransformListener mTfListener;

    bool mPublishCostMaps = false;
    bool mIsVerbose = false;

    void pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg);

public:
    CostMapNode();
};