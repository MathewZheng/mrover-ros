#pragma once

#include <iostream>
#include <optional>
#include <unistd.h>

#include <moteus/moteus.h>

#include <can_manager.hpp>
#include <controller.hpp>

using namespace mjbots;

enum class Mode {
    Stopped = 0,
    Fault = 1,
    PreparingToOperate1 = 2,
    PreparingToOperate2 = 3,
    PreparingToOperate3 = 4,
    PWMMode = 5,
    VoltageMode = 6,
    VoltageFOC = 7,
    VoltageDQ = 8,
    Current = 9,
    Position = 10,
    Timeout = 11,
    ZeroVelocity = 12,
    StayWithin = 13,
    MeasureInductance = 14,
    Brake = 15
};

enum class ErrorCode {
    DmaStreamTransferError = 1,
    DmaStreamFifiError = 2,
    UartOverrunError = 3,
    UartFramingError = 4,
    UartNoiseError = 5,
    UartBufferOverrunError = 6,
    UartParityError = 7,
    CalibrationFault = 32,
    MotorDriverFault = 33,
    OverVoltage = 34,
    EncoderFault = 35,
    MotorNotConfigured = 36,
    PwmCycleOverrun = 37,
    OverTemperature = 38,
    StartOutsideLimit = 39,
    UnderVoltage = 40,
    ConfigChanged = 41,
    ThetaInvalid = 42,
    PositionInvalid = 43,
};

class BrushlessController : public Controller {
public:
    void update(const std::vector<uint8_t>& frame) override;

    void set_desired_speed_unit(double speed) override; // from -1.0 to 1.0

    void set_desired_speed_rev_s(double speed); // in rev/s

    void set_desired_torque() override {
        moteus::Controller::Options options;
        options.id = 1;

        moteus::Controller controller(options);
        auto transport = moteus::Controller::MakeSingletonTransport({});


        // Command a stop to the controller in order to clear any faults.
        controller.SetStop();

        std::vector<moteus::CanFdFrame> send_frames;
        //std::vector<moteus::CanFdFrame> receive_frames;


        moteus::CurrentMode::Command cmd;

        TorqueModel dut(0.41f, 17.0f, 0.002f, 97.0f);
        const float calculated_current = dut.torque_to_current(torque);

        //        cmd.q_A = // TODO
        //        cmd.d_A = // TODO


        send_frames.push_back(controller.MakeCurrent(cmd));

        transport->BlockingCycle(&send_frames[0], send_frames.size(),
                                 &receive_frames);
    }

    void set_desired_position(int position) override {
        moteus::Controller::Options options;
        options.id = 1;

        moteus::Controller controller(options);

        // Command a stop to the controller in order to clear any faults.
        controller.SetStop();

        moteus::PositionMode::Command cmd;

        // Here we will just command a position of NaN and a velocity of
        // 0.0.  This means "hold position wherever you are".
        cmd.position = position;
        //cmd.velocity = 0.0;

        const auto maybe_result = controller.SetPositionWaitComplete(cmd, 0.01);

        //debug
        if (maybe_result) {
            const auto r = maybe_result->values;
            ::printf("%3d p/v/t=(%7.3f,%7.3f,%7.3f)  v/t/f=(%5.1f,%5.1f,%3d)   \r",
                     static_cast<int>(r.mode),
                     r.position,
                     r.velocity,
                     r.torque,
                     r.voltage,
                     r.temperature,
                     r.fault);
            ::fflush(stdout);
        }
    }

    MotorType get_type() override {
        return MotorType::Brushless;
    }

    void test_set_velocity(double speed) {

        // The following DefaultArgProcess is an optional call.  If made,
        // then command line arguments will be handled which allow setting
        // and configuring the default 'transport' to be used if none is
        // specified in Controller::Options::transport.
        //moteus::Controller::DefaultArgProcess(argc, argv);

        // There are many possible options to set for each controller
        // instance.  Here we re-set the ID to the default (1), just to show
        // how it is done.
        moteus::Controller::Options options;
        options.id = 1;

        moteus::Controller controller(options);
        auto transport = moteus::Controller::MakeSingletonTransport({});


        // Command a stop to the controller in order to clear any faults.
        controller.SetStop();

        std::vector<moteus::CanFdFrame> send_frames;
        std::vector<moteus::CanFdFrame> receive_frames;


        moteus::PositionMode::Command cmd;

        // Here we will just command a position of NaN and a velocity of
        // 0.0.  This means "hold position wherever you are".

        cmd.position = std::numeric_limits<double>::quiet_NaN();
        cmd.velocity = speed;


        send_frames.push_back(controller.MakePosition(cmd));

        transport->BlockingCycle(&send_frames[0], send_frames.size(),
                                 &receive_frames);
    }
    void set_desired_throttle(float throttle) override; // from -1.0 to 1.0
    void set_desired_velocity(float velocity) override; // in rev/s
    void set_desired_position(float position) override;

    BrushlessController(ros::NodeHandle& nh, const std::string& name) : Controller(nh, name) {
        torque = 0.3f;
    }
    ~BrushlessController() override = default;

private:
    std::optional<ErrorCode> err;
    float torque{};
};
