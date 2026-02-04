/*
////////////////////////////////////////////////////
mode.hpp - Defines our custom fixed wing flight mode

// NOTE: This is not our flight mission for MO 2/2.1. I made this exclusivly to learn how to write these.
// MAIN MO2/2.1 flight code can be found in undead.hpp

Bijan Jourabchi
University of Colorado Boulder
Undead Reckoning
////////////////////////////////////////////////////
*/

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>

#include <px4_ros2/control/setpoint_types/experimental/attitude.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <px4_ros2/odometry/local_position.hpp>

// Core control setpoints
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp> // Set desired attitude
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp> // Set desired thrust level

// Vehicle state feedback
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <cmath>


using namespace std::chrono_literals; // NOLINT

static const std::string kName = "TestFLIGHT";

class UASFlightMode : public px4_ros2::ModeBase
{
public:
    explicit UASFlightMode(rclcpp::Node & node) 
    : ModeBase(node, Settings{kName}),
    _node{node}
    {
        // Publishers for control
        _attitude_setpoint_pub = this->node().create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
            "/fmu/in/vehicle_attitude_setpoint",10);
        

        _thrust_setpoint_pub = this->node().create_publisher<px4_msgs::msg::VehicleThrustSetpoint>(
            "/fmu/in/vehicle_thrust_setpoint", 10);
        
        // Goto setpoints allow you to set a target position for the UAS to move 
        // to, along with option heading and max speed values.
        
        _vehicle_state = std::make_shared<px4_ros2::OdometryAttitude>(*this);

        _vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

        _attitude_sp_type = std::make_shared<px4_ros2::AttitudeSetpointType>(*this);
    }

    ~UASFlightMode() override {}

    void onActivate() override {
        RCLCPP_DEBUG(_node.get_logger(), "Fixed Wing Flight Mode Activated");

        _mode_start_time = _node.get_clock()->now();

        auto q = _vehicle_state->attitude();
        auto euler = quaternion_to_euler(q);
        _initial_yaw = static_cast<float>(euler[2]);

        _phase = Phase::STRAIGHT_1;

    }

    void onDeactivate() override {
        RCLCPP_DEBUG(_node.get_logger(), "Fixed Wing Flight Mode Deactivated");
    }

    void updateSetpoint(float dt) override {
        auto current_time = _node.get_clock()->now();
        double elapsed_time = (current_time - _mode_start_time).seconds();

        // State machine
        switch(_phase) {
            case Phase::STRAIGHT_1:
                if (elapsed_time > STRAIGHT_FLIGHT_DURATION) {

                    _phase = Phase::TURN;
                    _turn_start_quaternion = _vehicle_state->attitude();
                    _turn_start_yaw = heading_from_quaternion(_turn_start_quaternion);

                    // setup unwrapped trackers
                    _cmd_heading_unwrapped = _turn_start_yaw;
                    _start_heading_unwrapped = _turn_start_yaw;
                    _prev_heading = _turn_start_yaw;
                    _have_prev_heading = true;

                    RCLCPP_DEBUG(_node.get_logger(), "STRAIGHT FLIGHT COMPLETE, STARTING TURN");
                }
                fly_straight();
                break;
            case Phase::TURN:
                if(completed_turn()) {
                    _phase = Phase::STRAIGHT_2;
                    RCLCPP_DEBUG(_node.get_logger(), "TURN COMPLETE");
                }
                coordinated_turn(dt);
                break;
            case Phase::STRAIGHT_2:
                fly_straight();
                break;
        }

    }

private:

    enum class Phase {
        STRAIGHT_1 = 0,
        TURN,
        STRAIGHT_2
    } _phase;

    rclcpp::Node & _node;
    
    std::shared_ptr<px4_ros2::OdometryAttitude> _vehicle_state;
    std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
    std::shared_ptr<px4_ros2::AttitudeSetpointType> _attitude_sp_type;



    Eigen::Quaternionf _turn_start_quaternion;
    Eigen::Vector3f _position_m;

    // Can change as needed
    static constexpr double STRAIGHT_FLIGHT_DURATION = 30; // seconds
    static constexpr float CRUISE_PITCH = 0.05f; // radians
    static constexpr float CRUISE_THRUST = 0.6f; // 60% throttle
    static constexpr float TURN_BANK_ANGLE = 0.524f; // 45 degrees in rads
    static constexpr float TURN_THRUST = 0.65f; // 65% throttle
    static constexpr float TURN_SPEED = 25.0f; // m/s
    static constexpr float g = 9.81f; // m/s^2
    float _cmd_heading_unwrapped = 0.0f;
    float _start_heading_unwrapped = 0.0f;
    float _prev_heading = 0.0f;
    bool _have_prev_heading = false;

    void fly_straight() {
        
        // Slight pitch, constant heading
        auto q = euler_to_quaternion(0.0,CRUISE_PITCH,_initial_yaw);

        // Thrust Setpoint
        Eigen::Vector3f thrust_des{CRUISE_THRUST, 0.f, 0.f};

        _attitude_sp_type->update(
            q,
            thrust_des,
            0
        );

    }

    // Turn helped fxns

    static float normalize_angle(float a) {
        // normalize to [-pi, pi]
        while (a > M_PI) a-=2.0f * M_PI;
        while (a <= -M_PI) a+=2.0f*M_PI;
        return a;
    }


    float heading_from_quaternion(const Eigen::Quaternionf &q) {
        Eigen::Vector3f f = q * Eigen::Vector3f::UnitX();
        float heading = std::atan2(f.y(),f.x());
        return normalize_angle(heading);
    }

    static float unwrap_heading(float prev_wrapped, float new_wrapped) {
        float d = new_wrapped - prev_wrapped;
        // bring d into [-pi pi]
        if (d>M_PI) d -= 2.0f * M_PI;
        else if (d < -M_PI) d += 2.0f * M_PI;
        return prev_wrapped + d;
    }
    void coordinated_turn(float dt) {
        
        // Current Orientation
        auto q = _vehicle_state->attitude();
        float current_heading_wrapped = heading_from_quaternion(q);

        // yaw rate, fxn of bank angle
        const float TURN_RATE  = (g * std::tan(TURN_BANK_ANGLE)) / TURN_SPEED;

        // Unwrap for cumaltive rotation
        float current_heading_unwrapped;

        if (!_have_prev_heading) {
            current_heading_unwrapped = current_heading_wrapped;
            _have_prev_heading = true;
        } else {
            current_heading_unwrapped = unwrap_heading(_prev_heading, current_heading_wrapped);
        }
        _prev_heading = current_heading_unwrapped;

        // Advance command heading
        _cmd_heading_unwrapped += TURN_RATE * dt;

        // wrap to [-pi, pi] for autopilot
        float desired_heading_wrapped = normalize_angle(_cmd_heading_unwrapped);

        // Manage setpoints
        auto q_des = euler_to_quaternion(TURN_BANK_ANGLE,CRUISE_PITCH,desired_heading_wrapped);
        Eigen::Vector3f thrust_des{TURN_THRUST, 0.f, 0.f};

        _attitude_sp_type->update(
            q_des,
            thrust_des,
            TURN_RATE
        );
        

    }

    bool completed_turn() {
        auto q = _vehicle_state->attitude();
        float current_wrapped = heading_from_quaternion(q);
        float current_unwrapped = _have_prev_heading ? unwrap_heading(_prev_heading, current_wrapped) : current_wrapped;

        float delta = std::abs(current_unwrapped - _start_heading_unwrapped);

        const float TOLERNACE = 0.175f;
        return std::abs(delta - M_PI) < TOLERNACE;

    }



    template<typename Q>
    Eigen::Vector3d quaternion_to_euler(const Q & q_in) {
        Eigen::Quaterniond qd(
            static_cast<double>(q_in.w()),
            static_cast<double>(q_in.x()),
            static_cast<double>(q_in.y()),
            static_cast<double>(q_in.z())
        );
        if (qd.norm() > 0.0) { qd.normalize(); }
        auto euler = qd.toRotationMatrix().eulerAngles(2, 1, 0);
        return Eigen::Vector3d(euler[2], euler[1], euler[0]);
    }
 // ...exist

    Eigen::Quaternionf euler_to_quaternion(float roll, float pitch, float yaw) {
        // Use Eigen/Geometry library for euler to quaternion conversion. This is needed as quaternions are the way attitude setpoints are passed

        // Create AngleAxis objects for each rotation
        Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitchAngle(pitch,Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());

        // Combine to form our quaternion
         Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;
        return q;

    }

    // Publishers
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr _attitude_setpoint_pub;
    rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr _thrust_setpoint_pub;



    // Subscribers
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr _attitude_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr _local_position_sub;


    rclcpp::Time _mode_start_time;
    float _initial_yaw = 0.0f;
    float _turn_start_yaw = 0.0f;

};

class executeUAS : public px4_ros2::ModeExecutorBase
{
public:
      executeUAS(rclcpp::Node & node, px4_ros2::ModeBase & owned_mode)
    : ModeExecutorBase(node, {px4_ros2::ModeExecutorBase::Settings::Activation::ActivateImmediately}, owned_mode),
    _node(node)
    {
    }

    enum class State
    {
        Reset,
        Arming,
        TakingOff,
        FlightMode,
        WaitUntilDisarmed
    };

    void onActivate() override
    {
        runState(State::Arming, px4_ros2::Result::Success);
    }

    void onDeactivate(DeactivateReason reason) override
    {
    }

    void runState(State state, px4_ros2::Result previous_result)
    {
        if (previous_result != px4_ros2::Result::Success) {
            RCLCPP_ERROR(
                _node.get_logger(), "State %i: Previous state failed: %s, ABORTING", (int)state, resultToString(previous_result)
            );

            return;
        }

        RCLCPP_DEBUG(_node.get_logger(), "Executing State %i", (int)state);

        switch (state) {
            case State::Reset:
                break;
            case State::Arming:
                arm([this](px4_ros2::Result result) {runState(State::TakingOff, result);});
                break;
            case State::TakingOff:
                takeoff([this](px4_ros2::Result result) {runState(State::FlightMode, result);}, 50.0f);
                break;
            case State::FlightMode:
                scheduleMode(
                    ownedMode().id(), [this](px4_ros2::Result result) {
                        runState(State::WaitUntilDisarmed, result);
                    }
                );
                break;
            case State::WaitUntilDisarmed:
                waitUntilDisarmed(
                    [this](px4_ros2::Result result) {
                        RCLCPP_INFO(_node.get_logger(), "ALL STATES COMPLETE (%s)", resultToString(result));
                    }
                );
                break;
        }
    }
private:
    rclcpp::Node &_node;
};