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
    // fromTf is still erroring (something about it being inaccessible)
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
        // check here maybe to see if it exists already in costMap
        std::pair<int, int> currentPoint = convertToCell(point.x, point.y);
        for (int i = 0; i < (int)mCostMapPoints.size(); ++i) {
            std::pair<int, int> costMapPoint = convertToCell(mCostMapPoints[i].point.x(), mCostMapPoints[i].point.y());
            if (currentPoint == costMapPoint) {
                mCostMapPoints.erase(mCostMapPoints.begin() + i);
                break;
            }
        }
        mCostMapPoints.emplace_back(Eigen::Vector2f{point.x, point.y}, intCost, mFrameNumber);
        if (point.x < minx) {
            minx = point.x;
        }
        if (point.y < miny) {
            miny = point.y;
        }
    }

    // Should be done now: Change this to use resolution to calculate indices
    // Idea: for checking duplicates potentially keep track with a set?
    for (int i = 0; i < (int)mCostMapPoints.size(); ++i) {
        std::pair<int, int> currentPoint = convertToCell(mCostMapPoints[i].point.x(), mCostMapPoints[i].point.y());
    }

    if (mPublishCostMaps) {
        mCostMapPub.publish(mLocalGrid);
    }

    mFrameNumber++;
}

std::pair<int, int> CostMapNode::convertToCell(float pointx, float pointy) {
    // HERE: code for the previous way we calculated the indices
    // float differencex = pointx - minx;
    // float differencey = pointy - miny;
    // int indexx = floor(differencex);
    // int indexy = floor(differencey); 

    float resolution = mLocalGrid.info.resolution;
    // not sure if the width and height we are concerned with is the one in the mapmetadata for the occupancy grid or
    // the width and height specified above. for now i'm gonna assume its just the occupancy grid one
    uint32_t gridWidth = mLocalGrid.info.width;
    uint32_t gridHeight = mLocalGrid.info.height;
    return std::pair<int, int>(floor(pointx / resolution) + (gridWidth / 2), floor(pointy / resolution) + (gridHeight / 2));          
}
