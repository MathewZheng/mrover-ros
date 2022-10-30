

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <nav_msgs/OccupancyGrid.h>
#include <image_transport/image_transport.h>
#include <std_srvs/SetBool.h>

class CostMapNode {
private:
    ros::NodeHandle mNh;
    ros::NodeHandle mPnh;

    
    ros::Publisher mCostMapPub;
    ros::NodeHandle mCMt;

    ros::Subscriber mPcSub;
    tf2_ros::Buffer mTfBuffer;
    tf2_ros::TransformListener mTfListener;

    bool mPublishCostMaps = false;
    bool mIsVerbose = false;

    void pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg);
public:
    CostMapNode();
};