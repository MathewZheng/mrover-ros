#include "cost_map.hpp"
#include <pcl/features/normal_3d.h>

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
    SE3 pcTf;
    try {
        pcTf = SE3::fromTfTree(mTfBuffer, "map", "base_link");
    }
    catch(...) {
        ROS_WARN("Error reading fromTfTree");
    }
    Eigen::Vector3d translationDiff;
    Eigen::Quaterniond rotationDiff;
    if (mPreviousPose) {
        translationDiff = pcTf.positionVector() - mPreviousPose->positionVector();

        // TODO: rotationDiff should be from north instead of from previous pose because grid stays oriented North

    }
    // make 2d translation
    Eigen::Vector2d translationDiff2d = translationDiff.head<2>();

    // Clear scratch buffer
    for (auto &row: mCostMapPointsScratch) {
        for (auto &cost: row) {
            cost = std::nullopt;
        }
    }

    // Transform all current points
    for (size_t i = 0; i < mCostMapPoints.size(); ++i) {
        for (size_t j = 0; j < mCostMapPoints[i].size(); ++j) {
            //TODO: make auto and reference??
            // TODO: fill mCostMapPointsScratch in with transformed point with delta
            std::optional<CostMapPoint> currentPoint = mCostMapPoints[i][j];
            if (currentPoint) {
                // Don't fill in points that now lie outside
                Eigen::Vector2d newPoint = currentPoint.value().point - translationDiff2d;
                auto newCellIndex = convertToCell(newPoint);
                if (newCellIndex) {
                    //NOTE: should this be [i][j] or [newCell.first][newCell.second]?
                    auto [x, y] = newCellIndex.value();
                    mCostMapPointsScratch[x][y] = {newPoint, currentPoint->cost};
//                    mCostMapPointsScratch[x][y].emplace(newPoint, currentPoint->cost);
                }
            }
        }
    }

    // Add/replace new point cloud points
    pcl::fromROSMsg(*msg, *mCloudPtr);
    // pcl::NormalEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> ne;
    // ne.setInputCloud(mCloudPtr);
    // pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal> ());
    // ne.setSearchMethod(tree);
    // pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    // ne.setRadiusSearch(0.03);
    // ne.compute(*cloud_normals);


    for (size_t i = 0; i < (*mCloudPtr).size(); i++) {
        pcl::PointXYZRGBNormal &point = (*mCloudPtr)[i];
        // point.normal_x = ((*cloud_normals)[i].normal_x);
        // point.normal_y = ((*cloud_normals)[i].normal_y);
        // point.normal_z = ((*cloud_normals)[i].normal_z);
        Eigen::Vector3d xyz{point.x, point.y, point.z};
        xyz = pcTf.rotationQuaternion() * xyz;
        Eigen::Vector2d xy = xyz.head<2>();
        // Rotate point around rover by rover rotation diff

        //NOTE: should z coordinate be 0? changed to point.normal_z for now
        Eigen::Vector3d normal{point.normal_x, point.normal_y, point.normal_z};
        Eigen::Vector3d up{0.0f, 0.0f, 1.0f};

        float costValue = 1.0 - std::fabs(normal.dot(up));
        auto intCostValue = static_cast<int8_t>(std::lround(costValue * MAX_COST));
        auto currentCell = convertToCell(xy);
        if (currentCell) {
            auto [x, y] = currentCell.value();
            auto &cost = mCostMapPointsScratch[x][y];
            //NOTE: might be wrong
            // might need to convert normals to unit normals

            // Emplace needed to run first to initialize point in grid, would throw errors otherwise
            cost.emplace();
            cost->point = xy;
            cost->cost = intCostValue; 
        }

    }

    std::swap(mCostMapPoints, mCostMapPointsScratch);

    // Put costs from mCostMapPoints into mLocalGrid
    for (size_t i = 0; i < mCostMapPoints.size(); ++i) {
        for (size_t j = 0; j < mCostMapPoints[i].size(); ++j) {
            std::pair<size_t, size_t> currentPoint{i, j};
            auto &point = mCostMapPoints[currentPoint.first][currentPoint.second];
            if (point) {
                // std::cout << point->cost << " ";
                mLocalGrid.data[i * mCostMapPoints.size() + j] = point->cost;
            } else {
                //std::cout << -1 << " ";
                mLocalGrid.data[i * mCostMapPoints.size() + j] = -1;
            }

        }
        //std::cout << "\n";
    }

    if (mPublishCostMaps) {
        mCostMapPub.publish(mLocalGrid);
    }

    mFrameNumber++;
    mPreviousPose = pcTf;
}

std::optional<std::pair<size_t, size_t>> CostMapNode::convertToCell(Eigen::Vector2d const &point) {
    // TODO: return std::optional
    // HERE: code for the previous way we calculated the indices
    // float differencex = pointx - minx;
    // float differencey = pointy - miny;
    // int indexx = floor(differencex);
    // int indexy = floor(differencey); 

    uint32_t mHeight = 64, mWidth = 64;

    double resolution = 1;
    // not sure if the width and height we are concerned with is the one in the mapmetadata for the occupancy grid or
    // the width and height specified above. for now i'm gonna assume its just the occupancy grid one
    Eigen::Vector2d gridDimension{mWidth, mHeight};
    auto index = (point / resolution + gridDimension / 2).cast<size_t>();
    if (index.x() >= mHeight || index.y() >= mWidth) {
        return std::nullopt;
    } else {
        std::pair<size_t, size_t> cell = {index.x(), index.y()};
        return cell;
    }
}