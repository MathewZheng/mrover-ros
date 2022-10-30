#include "cost_map.hpp"

/*  Calculate the normal of each point xyz, store it in the last three channels, with message diagram:
 *  x (f32), y (f32), z (f32), rgba (uint32), normal_x (f32), normal_y (f32), normal_z (f32)
 *  
 *  Cost is calculated as:
 *  c(x,y) = 1 - abs(dot(unit(normal(x,y)), <0, 0, 1>))
 * 
*/ 
void CostMapNode::pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg) {
    //TODO complete calculation
    // idea, create channel iters, xyz (rgb unused), nx, ny, nz
    // iterate w these, publish into nx, ny, nz

    uint32_t mHeight = 64, mWidth = 64;

    std_msgs::Header();
    nav_msgs::MapMetaData();

    msg = nav_msgs::OccupancyGrid(std_msgs::Header();

    if(mPublishCostMaps){
        mCostMapPub.publish(msg);
    }

}
