#include "cost_map.hpp"

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

    if (mPublishCostMaps) {
        mCostMapPub.publish(mLocalGrid);
    }

    pcl::fromROSMsg(*msg, *mCloudPtr);
    for (pcl::PointXYZRGBNormal &point: *mCloudPtr) {
        float x = point.x;
        float y = point.y;
    }
}
