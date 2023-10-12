#include "pch.hpp"

namespace mrover {

    class CudaTagDetectorNodelet : public nodelet::Nodelet {
    private:
        ros::NodeHandle mNh, mPnh;

        void onInit() override;

    public:
        CudaTagDetectorNodelet() = default;

        ~CudaTagDetectorNodelet() override = default;
    };

} // namespace mrover
