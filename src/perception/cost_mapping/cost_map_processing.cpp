#include "cost_map.hpp"
#include "../se3.hpp"

#include <limits>

/*  Calculate the normal of each point xyz, store it in the last three channels, with message diagram:
 *  x (f32), y (f32), z (f32), rgba (uint32), normal_x (f32), normal_y (f32), normal_z (f32)
 *  
 *  Cost is calculated as:
 *  c(x,y) = 1 - abs(dot(unit(normal(x,y)), <0, 0, 1>))
 * 
*/
void CostMapNode::pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const &msg) {
    //TODO complete calculation
    // idea, create channel iters, xyz (rgb unused), nx, ny, nz
    // iterate w these, publish into nx, ny, nz
    uint32_t mHeight = 64, mWidth = 64;

    // 1. Get the transformation from the previous rover pose to the current pose

    // 2. Remove all points that are now too far away

    // 3. Use that transform to update all points in mCostMapPoints, the "point" vector

    // 4. Iterate mCloudPtr and add all new cost map calculations to mCostMapPoints

    // 5. For each point in mCostMapPoints, add to its correct "bin" or cell

    // 6. There is a chance there will be multiple, keep ony the one with the newest "frameNumberSeen"

    // 7. Actually create the cost map and put it into mLocalGrid, publish it

    geometry_msgs::TransformStamped pcTf = mTfBuffer.lookupTransform("map", "base_link", ros::Time(0));
    SE3 SE3Tf = SE3::fromTf(pcTf.transform);
    // TODO: Calculate transform using SE3

    float minx = mCloudPtr->at(0).x;
    float miny = mCloudPtr->at(0).y;
    pcl::fromROSMsg(*msg, *mCloudPtr);
    for (pcl::PointXYZRGBNormal &point: *mCloudPtr) {
        Eigen::Vector3f normal{point.normal_x, point.normal_y, 0.0f};
        Eigen::Vector3f up{0.0f, 0.0f, 1.0f};
        float cost = 1.0f - std::fabs(normal.dot(up));
        auto intCost = static_cast<uint8_t>(std::lround(cost) * std::numeric_limits<uint8_t>::max());
        mCostMapPoints.emplace_back(Eigen::Vector2f{point.x, point.y}, intCost, mFrameNumber);
        if (point.x < minx) {
            minx = point.x;
        }
        if (point.y < miny) {
            miny = point.y;
        }
    }

    // TODO: Change this to use resolution to calculate indices
    for (int i = 0; i < (int)mCostMapPoints.size(); ++i) {
        float pointx = mCostMapPoints[i].point.x();
        float pointy = mCostMapPoints[i].point.y();
        float differencex = pointx - minx;
        float differencey = pointy - miny;
        int indexx = floor(differencex);
        int indexy = floor(differencey);
    }

    if (mPublishCostMaps) {
        mCostMapPub.publish(mLocalGrid);
    }

    mFrameNumber++;
}
