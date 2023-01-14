#include "cost_map.hpp"

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
    // uint32_t mHeight = 64, mWidth = 64;

    SE3 pcTf = SE3::fromTfTree(mTfBuffer, "map", "base_link");
    // fromTf is still erroring (something about it being inaccessible)
    // TODO: calculate delta using last two poses (quintin: I can provide this)
    // Eigen::Quaterniond previousQuat = mPreviousPose.rotationQuaternion();
    // Eigen::Quaterniond currentQuat = pcTf.rotationQuaternion();
    // Eigen::Quaterniond rotationDiff = currentQuat * previousQuat.inverse();
    Eigen::Vector3d translationDiff = pcTf.positionVector() - mPreviousPose.positionVector();
    // make 2d translation
    Eigen::Vector2f translationDiff2d{translationDiff.x(), translationDiff.y()};
    SE3 delta;

    // Clear scratch buffer
    for (auto &row: mCostMapPointsScratch) {
        for (auto &cost: row) {
            cost = std::nullopt;
        }
    }

    // Transform all current points
    for (size_t i = 0; i < mCostMapPoints.size(); ++i) {
        for (size_t j = 0; i < mCostMapPoints[i].size(); ++j) {
            //TODO: make auto and reference??
            // TODO: fill mCostMapPointsScratch in with transformed point with delta
            std::optional<CostMapPoint> currentPoint = mCostMapPoints[i][j];
            if (currentPoint) {
                // Don't fill in points that now lie outside
                auto newCell = convertToCell(currentPoint.value().point + translationDiff2d);
                if (newCell) {        
                    //NOTE: should this be [i][j] or [newCell.first][newCell.second]?          
                    mCostMapPointsScratch[newCell.value().first][newCell.value().second].value().point = currentPoint.value().point + translationDiff2d;
                }
            }
            

        }
    }

    // Add/replace new point cloud points
    pcl::fromROSMsg(*msg, *mCloudPtr);
    for (pcl::PointXYZRGBNormal &point: *mCloudPtr) {
        Eigen::Vector2f xy{point.x, point.y};
        //NOTE: should z coordinate be 0? changed to point.normal_z for now
        Eigen::Vector3f normal{point.normal_x, point.normal_y, point.normal_z};
        Eigen::Vector3f up{0.0f, 0.0f, 1.0f};

        float costValue = 1.0f - std::fabs(normal.dot(up));
        auto intCostValue = static_cast<int8_t>(std::lround(costValue * MAX_COST));
        std::optional<std::pair<size_t, size_t>> currentCell = convertToCell(xy);
        
        if (currentCell) {
            auto &cost = mCostMapPointsScratch[currentCell.value().first][currentCell.value().second];
            //NOTE: might be wrong
            cost.value().point = xy;
            cost.value().cost = intCostValue;
        }
        
    }

    std::swap(mCostMapPoints, mCostMapPointsScratch);

    // Put costs from mCostMapPoints into mLocalGrid
    for (size_t i = 0; i < mCostMapPoints.size(); ++i) {
        for (size_t j = 0; i < mCostMapPoints[i].size(); ++j) {
            std::pair<size_t, size_t> currentPoint{i, j};
            auto &point = mCostMapPoints[currentPoint.first][currentPoint.second];
            if (point) {
                mLocalGrid.data[i*mCostMapPoints[i].size() + j] = point->cost;
            } else {
                mLocalGrid.data[i*mCostMapPoints[i].size() + j] = -1;
            }
            
        }
    }

    if (mPublishCostMaps) {
        mCostMapPub.publish(mLocalGrid);
    }

    mFrameNumber++;
    mPreviousPose = pcTf;
}

std::optional<std::pair<size_t, size_t>> CostMapNode::convertToCell(Eigen::Vector2f const &point) {
    // TODO: return std::optional
    // HERE: code for the previous way we calculated the indices
    // float differencex = pointx - minx;
    // float differencey = pointy - miny;
    // int indexx = floor(differencex);
    // int indexy = floor(differencey); 

    uint32_t mHeight = 64, mWidth = 64;

    float resolution = mLocalGrid.info.resolution;
    // not sure if the width and height we are concerned with is the one in the mapmetadata for the occupancy grid or
    // the width and height specified above. for now i'm gonna assume its just the occupancy grid one
    Eigen::Vector2f gridDimension{mLocalGrid.info.width, mLocalGrid.info.height};
    auto index = (point / resolution + gridDimension / 2).cast<size_t>();
    if (index.x() > mHeight || index.y() > mWidth) {
        return std::nullopt;
    } else {
        std::pair<size_t, size_t> cell = {index.x(), index.y()};
        return cell;
    }
}