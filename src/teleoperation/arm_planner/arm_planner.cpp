#include "arm_planner.hpp"

namespace mrover {

    // Constants

    // From: arm.urdf.xacro
    // TODO: get this dynamically from robot_description (parse URDF)
    constexpr double LINK_CHASSIS_ARM = 20;
    constexpr double LINK_AB = 22.8544;
    constexpr double LINK_BC = 19.5129;
    constexpr double LINK_CD = 5.59352;

    // Subscribers

    [[maybe_unused]] ros::Subscriber positionSubscriber;

    // Publishers

    // TODO: make a publisher for calculated velocity or position
    //       this is after making a trajectory and evaluating it at the current time

    // Private state

    // TODO: add private state

    int run(int argc, char** argv) {
        ros::init(argc, argv, "arm_planner");
        ros::NodeHandle nh;

        // TODO: add additional parameters
        double frequency{};
        nh.param<double>("/frequency", frequency, 100.0);

        positionSubscriber = nh.subscribe("arm_position_cmd", 1, positionCallback);

        ros::Rate rate{frequency};
        while (ros::ok()) {
            // TODO: add update loop

            rate.sleep();
            ros::spinOnce();
        }

        return EXIT_SUCCESS;
    }

    void positionCallback(Position const& positionMessage) {
        // TODO: given new position update trajectory state
    }

} // namespace mrover

int main(int argc, char** argv) {
    return mrover::run(argc, argv);
}