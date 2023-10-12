#include "cuda_tag_detector.hpp"

namespace mrover {

    void CudaTagDetectorNodelet::onInit() {
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle();
    }

} // namespace mrover

int main(int argc, char** argv) {
    ros::init(argc, argv, "cuda_tag_detector");

    // Start the ZED Nodelet
    nodelet::Loader nodelet;
    nodelet.load(ros::this_node::getName(), "mrover/CudaTagDetectorNodelet", ros::names::getRemappings(), {});

    ros::spin();

    return EXIT_SUCCESS;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrover::CudaTagDetectorNodelet, nodelet::Nodelet)
