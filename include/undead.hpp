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
#include <algorithm>
#include <limits>
#include <stdexcept>

#include <px4_ros2/control/setpoint_types/experimental/attitude.hpp>

// Odometry
#include <px4_ros2/odometry/attitude.hpp>
#include <px4_ros2/odometry/global_position.hpp>
#include <px4_ros2/odometry/airspeed.hpp>


// Vehicle state feedback
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>

using namespace std::chrono_literals; // NOLINT

static const std::string kname = "MOFLIGHT";

class MOFlightMode : public px4_ros2::ModeBase
{

public:
    explicit MOFlightMode(rclcpp::Node & node)
    : ModeBase(node, Settings{kname}),
    _node(node)
    {
        constexpr double kUnset = std::numeric_limits<double>::quiet_NaN();
        _final_latitude_deg = _node.declare_parameter<double>("final_latitude_deg", kUnset);
        _final_longitude_deg = _node.declare_parameter<double>("final_longitude_deg", kUnset);
        if (std::isnan(_final_latitude_deg) || std::isnan(_final_longitude_deg)) {
            throw std::runtime_error(
                "Missing required parameters final_latitude_deg/final_longitude_deg. "
                "Provide them via --ros-args --params-file <config.yaml>."
            );
        }

        // Subscribers for UAS state
        _vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);
        _vehicle_global_position = std::make_shared<px4_ros2::OdometryGlobalPosition>(*this);
        _vehicle_airspeed = std::make_shared<px4_ros2::OdometryAirspeed>(*this);

        // Attitude setpoint control
        _attitude_sp_type = std::make_shared<px4_ros2::AttitudeSetpointType>(*this);

    }


    ~MOFlightMode() override {}

    void onActivate() override
    {
        RCLCPP_DEBUG(_node.get_logger(), "Undead Reckoning Flight Activated");
        _phase = Phase::INIT;
        aligned = false;
        reachedGoal = false;
        _curr_throttle = {0.6f, 0.0f, 0.0f};
        _last_thrust_update = _node.get_clock()->now();

    }


    void onDeactivate() override {
        RCLCPP_DEBUG(_node.get_logger(), "Undead Reckoning Flight Mode Deactivated");

    }

    void updateSetpoint(float dt) override {

        // State Machine

        switch(_phase){
            case Phase::INIT:
            {
                _phase = Phase::CALIBRATION;
                _start_att = _vehicle_attitude->attitude();

                _curr_throttle = {0.6f,0.0f,0.0f};
                _des_pos_LL = {_final_latitude_deg, _final_longitude_deg, 0.0}; // Lat, Lon [deg], Alt [m]
                
                _ref_GPS = _vehicle_global_position->position();
                auto temp = gps_to_ned(_des_pos_LL, _ref_GPS);

                _des_pos_xy = {temp.x(),temp.y()};

                _mode_start = _node.get_clock()->now();

                break;
            }
            case Phase::CALIBRATION: 
            {
                auto curr_time = _node.get_clock()->now();

                if((curr_time - _mode_start).seconds() > 60.0) { // CHANGE
                    _phase = Phase::STRAIGHT;
                    RCLCPP_DEBUG(_node.get_logger(), "CALIBRATION LOOP COMPLETED");
                    break; // 180 second calibration
                }
                
                calibration(dt);
                break;
            }
            case Phase::STRAIGHT: 
                
                reached_();

                if (reachedGoal)
                {
                    _phase = Phase::DONE;
                    break;
                }

                is_aligned();

                if (aligned){
                    RCLCPP_DEBUG(_node.get_logger(), "ALIGNED: FLYING STRAIGHT");
                    fly_straight(dt);
                } else{
                    RCLCPP_DEBUG(_node.get_logger(), "ALIGNING");
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
        INIT = 0,
        CALIBRATION, 
        STRAIGHT,
        DONE 
    } _phase;

    rclcpp::Node & _node;

    // Member Variables

    std::shared_ptr<px4_ros2::AttitudeSetpointType> _attitude_sp_type; // Main control setpoint
    std::shared_ptr<px4_ros2::OdometryGlobalPosition> _vehicle_global_position; // Global position (NED)
    std::shared_ptr<px4_ros2::OdometryAttitude> _vehicle_attitude; // Attitude quaternion
    std::shared_ptr<px4_ros2::OdometryAirspeed> _vehicle_airspeed; // Airspeed (m/s)

    Eigen::Quaternionf _start_att; // self explanatory
    Eigen::Vector3d _ref_GPS;
    Eigen::Vector3f _curr_throttle; // curr throttle vector
    Eigen::Vector3d _des_pos_LL; 
    Eigen::Vector2f _des_pos_xy; // Des x,y position in NED [m]
    double _final_latitude_deg{};
    double _final_longitude_deg{};
    rclcpp::Time _last_thrust_update = _node.get_clock()->now();
    rclcpp::Time _last_pitch_update = _node.get_clock()->now();
    rclcpp::Time _mode_start = _node.get_clock()->now();
    bool aligned = false; // flag to determine if we are aligned in the direction we need to be traveling in to reach our desired position 
    bool reachedGoal = false;
    float PITCH = 0.05f; // rad GET A REAL ONE

    static constexpr float _max_speed_m_s = 13.f; // m/s
    static constexpr float TURN_BANK_ANGLE = 0.523599; // 30 degrees in rads
    static constexpr float g = 9.81f; // m/s^2
    static constexpr float GOAL_RADIUS = 1; // m
    static constexpr float MAX_ALT = 200.0f; // m, max alt
    static constexpr float kP_alt = 0.05f; // TUNE
    static constexpr float kP_thr = 0.05f; // TUNE

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

    void calibration(float dt)
    {
        float curr_yaw = _vehicle_attitude->yaw();
        float TURN_RATE  = (g * std::tan(TURN_BANK_ANGLE)) / _max_speed_m_s;
        _des_thrust(_curr_throttle);

        float cmd_yaw = curr_yaw + TURN_RATE*dt; // Ensures yaw and turn rate are consistant, controller won't work without it

        _attitude_sp_type->update(e2q(TURN_BANK_ANGLE,PITCH, cmd_yaw), // Attitude
            _curr_throttle, // Thrust
            TURN_RATE 
        );

    }
    
    void fly_straight(float dt)
    {
        
        // Fly straight at a fixed yaw
        // CRUISE_PITCH should be changed to be trim pitch angle

        _des_thrust(_curr_throttle); // updates _curr_throttle
        _alt_controller(PITCH);

        Eigen::Quaternionf _des_att = e2q(0.0,PITCH,_vehicle_attitude->yaw());

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

        // Apply turn rate in the direction of the error
        float cmd_turn_rate = 0.0f;
        float cmd_bank_angle = 0.0f;
        if (std::abs(yaw_error) > 0.01f) {  // Small threshold to avoid jitter
            float direction = (yaw_error > 0) ? 1.0f : -1.0f;
            // Apply direction to both Rate and Bank
            cmd_turn_rate = direction * TURN_RATE;
            cmd_bank_angle = direction * TURN_BANK_ANGLE;
        }

        _des_thrust(_curr_throttle); // updates _curr_throttle

        float cmd_yaw = curr_yaw + cmd_turn_rate*dt;
        //_des_pitch(PITCH);
        _attitude_sp_type->update(e2q(cmd_bank_angle,PITCH, cmd_yaw), // Attitude
            _curr_throttle, // Thrust
            cmd_turn_rate 
        );
    }

    void _des_thrust(Eigen::Vector3f & _curr_throttle){ // Get our desired thrust vector to maintain a 30 mph speed

        auto current_time = _node.get_clock()->now();

        if((current_time - _last_thrust_update).seconds() < 1.0) {
            return; // Exit throttle update if it hasn;t been 1 second yet.
        }

        // Update our timestamp
        _last_thrust_update = current_time;

        // auto _curr_throttle_x = _curr_throttle.x();
        auto _true_airspeed = _vehicle_airspeed->trueAirspeed();

        if (!std::isfinite(_true_airspeed)) {
            return;
        }

        // Keep a trim throttle and apply proportional correction around it.
        constexpr float kTrimThrottle = 0.6f;
        constexpr float kMinThrottle = 0.35f;
        float err = _max_speed_m_s - _true_airspeed;
        float cmd_thr = kTrimThrottle + (err * kP_thr);
        cmd_thr = std::clamp(cmd_thr, kMinThrottle, 1.0f);


        _curr_throttle = Eigen::Vector3f(cmd_thr,0,0);
    }

    /*
    * _alt_controller: pitch up or down to maintain a desired fixed altitude
    *
    */
    void _alt_controller(float & pitch) {

        Eigen::Vector3d LLA = _vehicle_global_position->position();
        Eigen::Vector3d NED = gps_to_ned(LLA,_ref_GPS);


        float curr_alt = -NED.z();
        float err = MAX_ALT - curr_alt;

        // Pitch Proporitonal Controller
        float cmd_pitch = err * kP_alt;

        float max_pitch = 0.26f; 
        PITCH = std::clamp(cmd_pitch, -max_pitch, max_pitch);
        
    }
 
    float _pos2heading(){ // Get a heading (yaw) in rads from our desired x,y NED pos
        
        Eigen::Vector3d LLA = _vehicle_global_position->position();
        Eigen::Vector3d curr_pos = gps_to_ned(LLA,_ref_GPS);

        // _des_pos_xy = [x (north), y (east)]

        // Delta position
        float x = _des_pos_xy.x() - curr_pos[0]; 
        float y = _des_pos_xy.y() - curr_pos[1];

        return atan2(y,x);

    }

    void reached_() {

        // Define a circle of radius 100 m (subject to change radius) around the goal
        Eigen::Vector3d LLA = _vehicle_global_position->position();
        Eigen::Vector3d curr_pos = gps_to_ned(LLA,_ref_GPS);

        float dx = _des_pos_xy.x() - curr_pos[0]; 
        float dy = _des_pos_xy.y() - curr_pos[1];
        
        // Essentially checking if the distance between a/c and the goal is less than 100 m
        float Dsqr = dx*dx + dy*dy;
        std::cout << Dsqr << std::endl;

        reachedGoal = (Dsqr < GOAL_RADIUS*GOAL_RADIUS);

    }

    
    /**
     * Converts Lat/Lon/Alt to NED position relative to a Home location.
     * * @param lat       Current Latitude (Degrees)
     * @param lon       Current Longitude (Degrees)
     * @param alt       Current Altitude (Meters)
     * @param ref_lat   Home/Origin Latitude (Degrees)
     * @param ref_lon   Home/Origin Longitude (Degrees)
     * @param ref_alt   Home/Origin Altitude (Meters)
     * @return          NED_Pos struct containing x(North), y(East), z(Down)
     */
    Eigen::Vector3d gps_to_ned(Eigen::Vector3d LLA, Eigen::Vector3d LLA_ref) {
        
        // Inputs
        float lat = LLA.x();
        float lon = LLA.y();
        float alt = LLA.z();

        float ref_lat = LLA_ref.x();
        float ref_lon = LLA_ref.y();
        float ref_alt = LLA_ref.z();

        // Constants
        const float EARTH_RADIUS = 6378137.0f; // Radius of Earth (meters)
        const float DEG2RAD = M_PI / 180.0f;

        // 1. Calculate Differences in Degrees
        float lat_err = lat - ref_lat;
        float lon_err = lon - ref_lon;
        float alt_err = alt - ref_alt;

        // 2. Convert to Radians (only for trig functions and lat/lon deltas)
        // Note: We convert the deltas to radians to multiply by Earth Radius
        float lat_rad = lat_err * DEG2RAD;
        float lon_rad = lon_err * DEG2RAD;

        // 3. Calculate North (X)
        // Distance along meridian: Arc length = radius * angle (in radians)
        double north_m = lat_rad * EARTH_RADIUS;

        // 4. Calculate East (Y)
        // Distance along parallel: Arc length = radius * angle * cos(latitude)
        // We use the reference latitude for the cosine scaling
        double east_m = lon_rad * EARTH_RADIUS * std::cos(ref_lat * DEG2RAD);

        // 5. Calculate Down (Z)
        // Down is negative altitude change
        double down_m = -alt_err;

        return Eigen::Vector3d(north_m, east_m, down_m);
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
