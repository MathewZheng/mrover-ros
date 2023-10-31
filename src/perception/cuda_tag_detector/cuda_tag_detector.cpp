#include "cuda_tag_detector.hpp"

namespace mrover {

    void CudaTagDetectorNodelet::onInit() {
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle();

        mImageSub = mPnh.subscribe<sensor_msgs::Image>("image", 1, &CudaTagDetectorNodelet::imageCallback, this);
    }

    void CudaTagDetectorNodelet::imageCallback(const sensor_msgs::ImageConstPtr& imageMessage) {
        // TODO: Upload this memory to the GPU
        // TODO: Run canny edge detection on the GPU OR adaptive thresholding
        // TODO: Do the rest of the paper stuff yk
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

#if MROVER_IS_NODELET
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrover::CudaTagDetectorNodelet, nodelet::Nodelet)
#endif
