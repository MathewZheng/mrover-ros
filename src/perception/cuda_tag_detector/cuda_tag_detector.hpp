#include "pch.hpp"

namespace mrover {

    class CudaTagDetectorNodelet : public nodelet::Nodelet {
    private:
        ros::NodeHandle mNh, mPnh;

        ros::Subscriber mImageSub;

        void onInit() override;

        void imageCallback(const sensor_msgs::ImageConstPtr& imageMessage);

    public:
        CudaTagDetectorNodelet() = default;

        ~CudaTagDetectorNodelet() override = default;
    };

} // namespace mrover
