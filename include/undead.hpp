/*
////////////////////////////////////////////////////
undead.hpp - Defines our custom fixed wing flight mode
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
#include <px4_ros2/utils/message_version.hpp>
#include <px4_ros2/common/setpoint_base.hpp>
#include <cmath>

#include <px4_ros2/control/setpoint_types/experimental/attitude.hpp>

// Odometry
#include <px4_ros2/odometry/attitude.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/odometry/airspeed.hpp>


// Vehicle state feedback
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>

//#include <px4_ros2/control/setpoint_types/goto.hpp>


using namespace std::chrono_literals; // NOLINT

static const std::string kname = "MOFLIGHT";

class MOFlightMode : public px4_ros2::ModeBase
{

public:
    explicit MOFlightMode(rclcpp::Node & node)
    : ModeBase(node, Settings{kname}),
    _node(node)
    {

        // Subscribers for UAS state
        _vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);
        _vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
        _vehicle_airspeed = std::make_shared<px4_ros2::OdometryAirspeed>(*this);

        // Attitude setpoint control
        _attitude_sp_type = std::make_shared<px4_ros2::AttitudeSetpointType>(*this);

    }


    ~MOFlightMode() override {}

    void onActivate() override
    {
        RCLCPP_DEBUG(_node.get_logger(), "Undead Reckoning Flight Activated");
        _phase = Phase::CALIBRATION;

    }


    void onDeactivate() override {
        RCLCPP_DEBUG(_node.get_logger(), "Undead Reckoning Flight Mode Deactivated");

    }

    void updateSetpoint(float dt) override {

        // State Machine

        switch(_phase){

            case Phase::CALIBRATION: {

                _phase = Phase::STRAIGHT;
                _start_att = _vehicle_attitude->attitude();
                _curr_throttle = {0.6f,0.0f,0.0f};
                _des_pos_xy = {2000.0f,1000.0f}; // North, East [m]
                break; }

                //calibration();

            case Phase::STRAIGHT: 
                
                reached_();

                if (reachedGoal)
                {
                    _phase = Phase::DONE;
                    break;
                }

                is_aligned();

                if (aligned){
                    fly_straight(dt);
                } else{
                    get_aligned(dt);
                }
                
                break;
            case Phase::DONE:
                completed(px4_ros2::Result::Success);
                break;
        }
    }

private:

    enum class Phase {
        CALIBRATION = 0, 
        STRAIGHT,
        DONE 
    } _phase;

    rclcpp::Node & _node;

    // Member Variables

    std::shared_ptr<px4_ros2::AttitudeSetpointType> _attitude_sp_type; // Main control setpoint
    std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position; // Local position (NED)
    std::shared_ptr<px4_ros2::OdometryAttitude> _vehicle_attitude; // Attitude quaternion
    std::shared_ptr<px4_ros2::OdometryAirspeed> _vehicle_airspeed; // Airspeed (m/s)

    Eigen::Quaternionf _start_att; // self explanatory
    Eigen::Vector3f _curr_throttle; // curr throttle vector
    Eigen::Vector2f _des_pos_xy; // Des x,y position in NED [m]
    rclcpp::Time _last_thrust_update = _node.get_clock()->now();
    bool aligned = false; // flag to determine if we are aligned in the direction we need to be traveling in to reach our desired position 
    bool reachedGoal = false;

    static constexpr float _max_speed_m_s = 13.f; // m/s
    static constexpr float CRUISE_PITCH = 0.05f; // rad GET A REAL ONE
    static constexpr float TURN_BANK_ANGLE = 0.524f; // 45 degrees in rads
    static constexpr float g = 9.81f; // m/s^2
    static constexpr float GOAL_RADIUS = 100; // m

    template<typename Q>

    Eigen::Vector3d q2e(const Q & q_in) { // quaternion to euler

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



    Eigen::Quaternionf e2q(float roll, float pitch, float yaw) { // euler to quaternion

        // Use Eigen/Geometry library for euler to quaternion conversion. This is needed as quaternions are the way attitude setpoints are passed

        // Create AngleAxis objects for each rotation
        Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitchAngle(pitch,Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());


        // Combine to form our quaternion
        Eigen::Quaternionf q = yawAngle * pitchAngle * rollAngle;
        return q;

    };


    void fly_straight(float dt)
    {
        
        // Fly straight at a fixed yaw
        // CRUISE_PITCH should be changed to be trim pitch angle
        Eigen::Quaternionf _des_att = e2q(0.0,CRUISE_PITCH,_vehicle_attitude->yaw());

        _des_thrust(_curr_throttle); // updates _curr_throttle

        _attitude_sp_type->update(_des_att, // Attitude
            _curr_throttle // Thrust
        );
        
    };

    void is_aligned()
    {
        float curr_yaw = _vehicle_attitude->yaw();
        float des_yaw = _pos2heading();

        if (abs(curr_yaw - des_yaw) < 0.1)
        {
            aligned = true;
            return;
        } else {
            aligned = false;
            return;
        }
    }

    void get_aligned(float dt){
        
        // Get current state
        float curr_yaw = _vehicle_attitude->yaw();
        float des_yaw = _pos2heading();

        float yaw_error = des_yaw - curr_yaw;
        yaw_error = atan2(sin(yaw_error), cos(yaw_error));  // wrap to [-pi, pi]
        // yaw rate, fxn of bank angle
        float TURN_RATE  = (g * std::tan(TURN_BANK_ANGLE)) / _max_speed_m_s;

        if (yaw_error < 0) TURN_RATE = -abs(TURN_RATE);  // Turn Right
        if (yaw_error > 0) TURN_RATE = abs(TURN_RATE);  // Turn Left

        _des_thrust(_curr_throttle); // updates _curr_throttle

        float cmd_yaw = curr_yaw + TURN_RATE*dt;

        _attitude_sp_type->update(e2q(TURN_BANK_ANGLE,CRUISE_PITCH, cmd_yaw), // Attitude
            _curr_throttle, // Thrust
            TURN_RATE 
        );
    }

    void _des_thrust(Eigen::Vector3f & _curr_throttle){ // Get our desired thrust vector to maintain a 30 mph speed

        auto current_time = _node.get_clock()->now();

        if((current_time - _last_thrust_update).seconds() < 1.0) {
            return; // Exit throttle update if it hasn;t been 1 second yet.
        }

        // Update our timestamp
        _last_thrust_update = current_time;

        auto _curr_throttle_x = _curr_throttle.x();
        auto _true_airspeed = _vehicle_airspeed->trueAirspeed();

        if (_true_airspeed > _max_speed_m_s) // REDUCE
        {
            _curr_throttle_x -= 0.05f;
        } else { // INCREASE
            _curr_throttle_x += 0.05f;
        }

        // CLAMP, never let throttle go above 1, below 0
        if (_curr_throttle_x > 1.0) _curr_throttle_x = 1.0f;
        if (_curr_throttle_x < 0) _curr_throttle_x = 0.0f;

        _curr_throttle = Eigen::Vector3f(_curr_throttle_x,0,0);
    }

    float _pos2heading(){ // Get a heading (yaw) in rads from our desired x,y NED pos
        
        auto curr_pos = _vehicle_local_position->positionNed();

        // _des_pos_xy = [x (north), y (east)]

        // Delta position
        float x = _des_pos_xy.x() - curr_pos[0]; 
        float y = _des_pos_xy.y() - curr_pos[1];

        return atan2(y,x);

    }

    void reached_() {

        // Define a circle of radius 100 m (subject to change radius) around the goal
        auto curr_pos = _vehicle_local_position->positionNed();
        float dx = _des_pos_xy.x() - curr_pos[0]; 
        float dy = _des_pos_xy.y() - curr_pos[1];
        
        // Essentially checking if the distance between a/c and the goal is less than 100 m
        float Dsqr = dx*dx + dy*dy;
        std::cout << Dsqr << std::endl;

        if (Dsqr < GOAL_RADIUS*GOAL_RADIUS) reachedGoal = true;

    }
};



class executeMOFlightMode : public px4_ros2::ModeExecutorBase
{
public:

    executeMOFlightMode(rclcpp::Node & node, px4_ros2::ModeBase & owned_mode)
    : ModeExecutorBase(node, {px4_ros2::ModeExecutorBase::Settings::Activation::ActivateImmediately}, owned_mode),
    _node(node)
    {
    }


    enum class State
    {
        Reset,
        Arming,
        FlightMode,
        Land,
        WaitUntilDisarmed
    };

    void onActivate() override
    {
        runState(State::FlightMode, px4_ros2::Result::Success);
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
                arm([this](px4_ros2::Result result) {runState(State::FlightMode, result);});
                break;
            case State::FlightMode:
                scheduleMode(
                    ownedMode().id(), [this](px4_ros2::Result result) {
                        runState(State::Land, result);
                    }
                );
                break;
            case State::Land:
                land([this](px4_ros2::Result result) {runState(State::WaitUntilDisarmed, result);});
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

