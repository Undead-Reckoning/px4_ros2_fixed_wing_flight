/*
////////////////////////////////////////////////////
undead_test.hpp - Test custom fixed wing flight mode
Based on undead.hpp mission architecture.
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
#include <px4_ros2/odometry/global_position.hpp>
#include <px4_ros2/odometry/airspeed.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

using namespace std::chrono_literals;  // NOLINT

static const std::string kTestModeName = "UNDEAD_TEST";

class UndeadTestMode : public px4_ros2::ModeBase
{
public:
    explicit UndeadTestMode(rclcpp::Node & node)
    : ModeBase(node, Settings{kTestModeName}),
      _node(node)
    {
        loadParameters();

        _vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);
        _vehicle_global_position = std::make_shared<px4_ros2::OdometryGlobalPosition>(*this);
        _vehicle_airspeed = std::make_shared<px4_ros2::OdometryAirspeed>(*this);
        _attitude_sp_type = std::make_shared<px4_ros2::AttitudeSetpointType>(*this);
    }

    ~UndeadTestMode() override = default;

    void onActivate() override
    {
        _phase = Phase::INIT;
        _curr_throttle = Eigen::Vector3f(kTrimThrottle, 0.0f, 0.0f);
        _mode_start = _node.get_clock()->now();
        _last_thrust_update = _mode_start;

        RCLCPP_INFO(_node.get_logger(), "UNDEAD_TEST activated");
    }

    void onDeactivate() override
    {
        RCLCPP_INFO(_node.get_logger(), "UNDEAD_TEST deactivated");
    }

    void updateSetpoint(float dt) override
    {
        switch (_phase) {
            case Phase::INIT:
                if (!initializeGeometry()) {
                    completed(px4_ros2::Result::ModeFailureOther);
                    _phase = Phase::DONE;
                    break;
                }
                _phase = Phase::GO_TO_START;
                break;

            case Phase::GO_TO_START:
                if (commandStartPointAcquire(dt)) {
                    _segment = Segment::LINE_1;
                    _phase = Phase::FLY;
                }
                break;

            case Phase::FLY:
                if ((_node.get_clock()->now() - _mode_start).seconds() >= _flight_time_s) {
                    RCLCPP_INFO(_node.get_logger(), "UNDEAD_TEST time complete");
                    _phase = Phase::DONE;
                    break;
                }

                updateSegmentProgress();
                commandCurrentSegment(dt);
                break;

            case Phase::DONE:
                completed(px4_ros2::Result::Success);
                break;
        }
    }

private:
    enum class Phase {
        INIT = 0,
        GO_TO_START,
        FLY,
        DONE,
    } _phase{Phase::INIT};

    enum class Segment {
        LINE_1 = 0,
        ARC_1,
        LINE_2,
        ARC_2,
    } _segment{Segment::LINE_1};

    struct ArcDef {
        Eigen::Vector2f center{};
        float radius_m{1.0f};
        float start_ang_rad{0.0f};
        float end_ang_rad{0.0f};
        float direction{1.0f};  // +1 CCW, -1 CW
        float total_progress_rad{0.0f};
    };

    rclcpp::Node & _node;

    std::shared_ptr<px4_ros2::AttitudeSetpointType> _attitude_sp_type;
    std::shared_ptr<px4_ros2::OdometryGlobalPosition> _vehicle_global_position;
    std::shared_ptr<px4_ros2::OdometryAttitude> _vehicle_attitude;
    std::shared_ptr<px4_ros2::OdometryAirspeed> _vehicle_airspeed;

    std::array<Eigen::Vector3d, 4> _rectangle_lla{};
    std::array<Eigen::Vector2f, 4> _track_ned{};
    ArcDef _arc1{};
    ArcDef _arc2{};

    double _flight_time_s{180.0};
    double _max_speed_m_s{13.0};
    double _max_alt_m{200.0};

    static constexpr float kTurnBankAngleRad = 0.523599f;  // 30 deg
    static constexpr float kPi = 3.14159265358979323846f;
    static constexpr float kHalfPi = 1.57079632679489661923f;
    static constexpr float kGravity = 9.81f;
    static constexpr float kPAlt = 0.05f;
    static constexpr float kPThr = 0.05f;
    static constexpr float kTrimThrottle = 0.6f;
    static constexpr float kMinThrottle = 0.35f;
    static constexpr float kMaxPitchRad = 0.26f;
    static constexpr float kLineCaptureMeters = 8.0f;
    static constexpr float kArcCaptureRad = 0.08f;
    static constexpr float kArcEndCaptureMeters = 15.0f;
    static constexpr float kArcRadiusToleranceMeters = 15.0f;
    static constexpr float kStartCaptureMeters = 1.0f;

    Eigen::Vector3f _curr_throttle{0.6f, 0.0f, 0.0f};
    float _pitch_cmd_rad{0.05f};
    Eigen::Vector3d _ref_gps{};

    rclcpp::Time _mode_start{0, 0, RCL_ROS_TIME};
    rclcpp::Time _last_thrust_update{0, 0, RCL_ROS_TIME};

    static float wrapAngle(float a)
    {
        return std::atan2(std::sin(a), std::cos(a));
    }

    static float segmentDistance(const Eigen::Vector2f & a, const Eigen::Vector2f & b)
    {
        return (b - a).norm();
    }

    static Eigen::Quaternionf e2q(float roll, float pitch, float yaw)
    {
        Eigen::AngleAxisf roll_angle(roll, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf pitch_angle(pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf yaw_angle(yaw, Eigen::Vector3f::UnitZ());
        return yaw_angle * pitch_angle * roll_angle;
    }

    void loadParameters()
    {
        const std::vector<double> empty_default{};
        auto lat = _node.declare_parameter<std::vector<double>>("rectangle_lat_deg", empty_default);
        auto lon = _node.declare_parameter<std::vector<double>>("rectangle_lon_deg", empty_default);
        _flight_time_s = _node.declare_parameter<double>("flight_time_s", 180.0);
        _max_speed_m_s = _node.declare_parameter<double>("max_speed_m_s", 13.0);
        _max_alt_m = _node.declare_parameter<double>("max_alt_m", 200.0);

        if (lat.size() != 4 || lon.size() != 4) {
            throw std::runtime_error("rectangle_lat_deg and rectangle_lon_deg must each contain exactly 4 values.");
        }

        if (_flight_time_s <= 0.0 || _max_speed_m_s <= 0.0) {
            throw std::runtime_error("flight_time_s and max_speed_m_s must be > 0.");
        }

        for (size_t i = 0; i < 4; ++i) {
            _rectangle_lla[i] = Eigen::Vector3d(lat[i], lon[i], 0.0);
        }
    }

    Eigen::Vector3d gpsToNed(const Eigen::Vector3d & lla, const Eigen::Vector3d & lla_ref) const
    {
        const float lat = static_cast<float>(lla.x());
        const float lon = static_cast<float>(lla.y());
        const float alt = static_cast<float>(lla.z());

        const float ref_lat = static_cast<float>(lla_ref.x());
        const float ref_lon = static_cast<float>(lla_ref.y());
        const float ref_alt = static_cast<float>(lla_ref.z());

        constexpr float earth_radius = 6378137.0f;
        constexpr float deg2rad = kPi / 180.0f;

        const float lat_err = lat - ref_lat;
        const float lon_err = lon - ref_lon;
        const float alt_err = alt - ref_alt;

        const float lat_rad = lat_err * deg2rad;
        const float lon_rad = lon_err * deg2rad;

        const double north_m = lat_rad * earth_radius;
        const double east_m = lon_rad * earth_radius * std::cos(ref_lat * deg2rad);
        const double down_m = -alt_err;

        return Eigen::Vector3d(north_m, east_m, down_m);
    }

    bool initializeGeometry()
    {
        _ref_gps = _vehicle_global_position->position();

        if (!std::isfinite(_ref_gps.x()) || !std::isfinite(_ref_gps.y())) {
            RCLCPP_ERROR(_node.get_logger(), "Invalid reference GPS at activation");
            return false;
        }

        for (size_t i = 0; i < 4; ++i) {
            const Eigen::Vector3d ned3 = gpsToNed(_rectangle_lla[i], _ref_gps);
            _track_ned[i] = Eigen::Vector2f(static_cast<float>(ned3.x()), static_cast<float>(ned3.y()));
        }

        // Re-index so edge (0->1) is one of the long edges.
        const float d01 = segmentDistance(_track_ned[0], _track_ned[1]);
        const float d12 = segmentDistance(_track_ned[1], _track_ned[2]);
        if (d12 > d01) {
            std::array<Eigen::Vector2f, 4> rot{};
            for (size_t i = 0; i < 4; ++i) {
                rot[i] = _track_ned[(i + 1U) % 4U];
            }
            _track_ned = rot;
        }

        const float long_1 = segmentDistance(_track_ned[0], _track_ned[1]);
        const float short_1 = segmentDistance(_track_ned[1], _track_ned[2]);
        if (long_1 < short_1) {
            RCLCPP_ERROR(_node.get_logger(), "Input points are not ordered as a rectangle perimeter.");
            return false;
        }

        _arc1 = buildArc(_track_ned[1], _track_ned[2], headingLine(_track_ned[0], _track_ned[1]));
        _arc2 = buildArc(_track_ned[3], _track_ned[0], headingLine(_track_ned[2], _track_ned[3]));

        return true;
    }

    float headingLine(const Eigen::Vector2f & start, const Eigen::Vector2f & end) const
    {
        const Eigen::Vector2f d = end - start;
        return std::atan2(d.y(), d.x());
    }

    ArcDef buildArc(const Eigen::Vector2f & start, const Eigen::Vector2f & end, float incoming_heading) const
    {
        ArcDef arc{};
        arc.center = 0.5f * (start + end);
        arc.radius_m = 0.5f * (end - start).norm();
        arc.start_ang_rad = std::atan2(start.y() - arc.center.y(), start.x() - arc.center.x());
        arc.end_ang_rad = std::atan2(end.y() - arc.center.y(), end.x() - arc.center.x());

        const float tangent_ccw = wrapAngle(arc.start_ang_rad + kHalfPi);
        const float tangent_cw = wrapAngle(arc.start_ang_rad - kHalfPi);

        const float err_ccw = std::fabs(wrapAngle(tangent_ccw - incoming_heading));
        const float err_cw = std::fabs(wrapAngle(tangent_cw - incoming_heading));

        arc.direction = (err_ccw <= err_cw) ? 1.0f : -1.0f;

        if (arc.direction > 0.0f) {
            arc.total_progress_rad = wrapToPositive(arc.end_ang_rad - arc.start_ang_rad);
        } else {
            arc.total_progress_rad = wrapToPositive(arc.start_ang_rad - arc.end_ang_rad);
        }

        // Keep the connector as a half-circle even with small numerical offsets.
        arc.total_progress_rad = kPi;
        return arc;
    }

    static float wrapToPositive(float a)
    {
        while (a < 0.0f) {
            a += 2.0f * kPi;
        }
        while (a >= 2.0f * kPi) {
            a -= 2.0f * kPi;
        }
        return a;
    }

    Eigen::Vector2f currentPositionNed2D() const
    {
        const Eigen::Vector3d lla = _vehicle_global_position->position();
        const Eigen::Vector3d ned = gpsToNed(lla, _ref_gps);
        return Eigen::Vector2f(static_cast<float>(ned.x()), static_cast<float>(ned.y()));
    }

    void updateSegmentProgress()
    {
        const Eigen::Vector2f pos = currentPositionNed2D();

        switch (_segment) {
            case Segment::LINE_1: {
                const float remain = (_track_ned[1] - pos).norm();
                if (remain <= kLineCaptureMeters) {
                    _segment = Segment::ARC_1;
                }
                break;
            }
            case Segment::ARC_1: {
                if (arcNearComplete(pos, _arc1)) {
                    _segment = Segment::LINE_2;
                }
                break;
            }
            case Segment::LINE_2: {
                const float remain = (_track_ned[3] - pos).norm();
                if (remain <= kLineCaptureMeters) {
                    _segment = Segment::ARC_2;
                }
                break;
            }
            case Segment::ARC_2: {
                if (arcNearComplete(pos, _arc2)) {
                    _segment = Segment::LINE_1;
                }
                break;
            }
        }
    }

    bool arcNearComplete(const Eigen::Vector2f & pos, const ArcDef & arc) const
    {
        const float theta_now = std::atan2(pos.y() - arc.center.y(), pos.x() - arc.center.x());
        float progress = 0.0f;

        if (arc.direction > 0.0f) {
            progress = wrapToPositive(theta_now - arc.start_ang_rad);
        } else {
            progress = wrapToPositive(arc.start_ang_rad - theta_now);
        }

        const Eigen::Vector2f end_pt(
            arc.center.x() + arc.radius_m * std::cos(arc.end_ang_rad),
            arc.center.y() + arc.radius_m * std::sin(arc.end_ang_rad));
        const float dist_to_end = (end_pt - pos).norm();
        const float radius_err = std::fabs((pos - arc.center).norm() - arc.radius_m);
        RCLCPP_DEBUG(_node.get_logger(), "progress: %.2f, dist_to_end: %.2f, radius_err: %.2f",
            progress, dist_to_end, radius_err);
        RCLCPP_DEBUG(_node.get_logger(), "arc.total_progress_rad - kArcCaptureRad: %.2f, kArcEndCaptureMeters: %.2f, kArcRadiusToleranceMeters: %.2f",
            arc.total_progress_rad - kArcCaptureRad, kArcEndCaptureMeters, kArcRadiusToleranceMeters);
        return progress >= (arc.total_progress_rad - kArcCaptureRad) &&
               dist_to_end <= kArcEndCaptureMeters &&
               radius_err <= kArcRadiusToleranceMeters;
    }

    void commandCurrentSegment(float dt)
    {
        updateThrottleHold();
        updateAltitudeHold();

        float yaw_cmd = _vehicle_attitude->yaw();
        float yaw_rate_cmd = 0.0f;
        float bank_cmd = 0.0f;
        bool use_yaw_rate = false;

        switch (_segment) {
            case Segment::LINE_1:
                yaw_cmd = headingLine(currentPositionNed2D(), _track_ned[1]);
                break;
            case Segment::ARC_1:
                computeArcCommand(_arc1, dt, yaw_cmd, yaw_rate_cmd, bank_cmd, use_yaw_rate);
                break;
            case Segment::LINE_2:
                yaw_cmd = headingLine(currentPositionNed2D(), _track_ned[3]);
                break;
            case Segment::ARC_2:
                computeArcCommand(_arc2, dt, yaw_cmd, yaw_rate_cmd, bank_cmd, use_yaw_rate);
                break;
        }

        if (!use_yaw_rate) {
            const float yaw_err = wrapAngle(yaw_cmd - _vehicle_attitude->yaw());
            const float direction = (yaw_err >= 0.0f) ? 1.0f : -1.0f;
            bank_cmd = direction * std::min(std::fabs(yaw_err), kTurnBankAngleRad);
            yaw_rate_cmd = direction * turnRateForSpeed();
            yaw_cmd = _vehicle_attitude->yaw() + yaw_rate_cmd * dt;
            use_yaw_rate = true;
        }

        const Eigen::Quaternionf att_sp = e2q(bank_cmd, _pitch_cmd_rad, yaw_cmd);

        if (use_yaw_rate) {
            _attitude_sp_type->update(att_sp, _curr_throttle, yaw_rate_cmd);
        } else {
            _attitude_sp_type->update(att_sp, _curr_throttle);
        }
    }

    bool commandStartPointAcquire(float dt)
    {
        const Eigen::Vector2f pos = currentPositionNed2D();
        const float dist_to_start = (_track_ned[0] - pos).norm();

        if (dist_to_start <= kStartCaptureMeters) {
            RCLCPP_INFO(_node.get_logger(), "Reached start waypoint, entering rectangle loop");
            return true;
        }

        updateThrottleHold();
        updateAltitudeHold();

        const float yaw_cmd = headingLine(pos, _track_ned[0]);
        const float yaw_err = wrapAngle(yaw_cmd - _vehicle_attitude->yaw());
        constexpr float kYawRatePGain = 1.5f;  // [rad/s] per [rad] heading error
        const float max_yaw_rate = turnRateForSpeed();
        const float yaw_rate_cmd = std::clamp(kYawRatePGain * yaw_err, -max_yaw_rate, max_yaw_rate);
        const float bank_cmd = std::clamp(
            (yaw_rate_cmd / max_yaw_rate) * kTurnBankAngleRad,
            -kTurnBankAngleRad,
            kTurnBankAngleRad);
        const float yaw_sp = _vehicle_attitude->yaw() + yaw_rate_cmd * dt;

        const Eigen::Quaternionf att_sp = e2q(bank_cmd, _pitch_cmd_rad, yaw_sp);
        _attitude_sp_type->update(att_sp, _curr_throttle, yaw_rate_cmd);
        return false;
    }

    void computeArcCommand(
        const ArcDef & arc,
        float dt,
        float & yaw_cmd,
        float & yaw_rate_cmd,
        float & bank_cmd,
        bool & use_yaw_rate) const
    {
        const Eigen::Vector2f pos = currentPositionNed2D();
        const Eigen::Vector2f rel = pos - arc.center;
        const float radius = std::max(arc.radius_m, 1.0f);
        const float r_now = std::max(rel.norm(), 1.0f);
        const float theta = std::atan2(pos.y() - arc.center.y(), pos.x() - arc.center.x());
        const float tangent_heading = wrapAngle(theta + arc.direction * kHalfPi);
        const float radial_err = r_now - radius;

        // Orbit guidance: bias tangent heading inward/outward based on radial error.
        constexpr float kOrbitGain = 3.0f;
        const float heading_correction = std::atan(kOrbitGain * radial_err / radius);
        const float desired_heading = wrapAngle(tangent_heading + arc.direction * heading_correction);

        float speed_m_s = static_cast<float>(_max_speed_m_s);
        const float tas = _vehicle_airspeed->trueAirspeed();
        if (std::isfinite(tas) && tas > 1.0f) {
            speed_m_s = tas;
        }

        const float max_rate_from_bank = (kGravity * std::tan(kTurnBankAngleRad)) / std::max(speed_m_s, 1.0f);
        const float yaw_rate_ff = arc.direction * (speed_m_s / radius);
        const float yaw_err = wrapAngle(desired_heading - _vehicle_attitude->yaw());
        constexpr float kYawRateHeadingGain = 1.5f;
        const float yaw_rate_fb = kYawRateHeadingGain * yaw_err;
        const float commanded_yaw_rate = std::clamp(yaw_rate_ff + yaw_rate_fb, -max_rate_from_bank, max_rate_from_bank);

        yaw_rate_cmd = commanded_yaw_rate;
        const float commanded_bank = std::atan((speed_m_s * yaw_rate_cmd) / kGravity);
        bank_cmd = std::clamp(commanded_bank, -kTurnBankAngleRad, kTurnBankAngleRad);
        yaw_cmd = _vehicle_attitude->yaw() + yaw_rate_cmd * dt;
        use_yaw_rate = true;
    }

    float turnRateForSpeed() const
    {
        return (kGravity * std::tan(kTurnBankAngleRad)) / static_cast<float>(_max_speed_m_s);
    }

    void updateThrottleHold()
    {
        const auto now = _node.get_clock()->now();
        if ((now - _last_thrust_update).seconds() < 1.0) {
            return;
        }
        _last_thrust_update = now;

        const float tas = _vehicle_airspeed->trueAirspeed();
        if (!std::isfinite(tas)) {
            return;
        }

        float cmd = kTrimThrottle + static_cast<float>(_max_speed_m_s - tas) * kPThr;
        cmd = std::clamp(cmd, kMinThrottle, 1.0f);
        _curr_throttle = Eigen::Vector3f(cmd, 0.0f, 0.0f);
    }

    void updateAltitudeHold()
    {
        const Eigen::Vector3d lla = _vehicle_global_position->position();
        const Eigen::Vector3d ned = gpsToNed(lla, _ref_gps);
        const float curr_alt = static_cast<float>(-ned.z());

        const float err = static_cast<float>(_max_alt_m) - curr_alt;
        const float cmd_pitch = err * kPAlt;
        _pitch_cmd_rad = std::clamp(cmd_pitch, -kMaxPitchRad, kMaxPitchRad);
    }
};

class executeUndeadTestMode : public px4_ros2::ModeExecutorBase
{
public:
    executeUndeadTestMode(rclcpp::Node & node, px4_ros2::ModeBase & owned_mode)
    : ModeExecutorBase(node, {px4_ros2::ModeExecutorBase::Settings::Activation::ActivateImmediately}, owned_mode),
      _node(node)
    {
    }

    enum class State {
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
        (void)reason;
    }

    void runState(State state, px4_ros2::Result previous_result)
    {
        if (previous_result != px4_ros2::Result::Success) {
            RCLCPP_ERROR(
                _node.get_logger(), "State %i failed: %s", static_cast<int>(state), resultToString(previous_result));
            return;
        }

        switch (state) {
            case State::FlightMode:
                scheduleMode(ownedMode().id(), [this](px4_ros2::Result result) { runState(State::Land, result); });
                break;
            case State::Land:
                land([this](px4_ros2::Result result) { runState(State::WaitUntilDisarmed, result); });
                break;
            case State::WaitUntilDisarmed:
                waitUntilDisarmed([this](px4_ros2::Result result) {
                    RCLCPP_INFO(_node.get_logger(), "UNDEAD_TEST complete (%s)", resultToString(result));
                });
                break;
        }
    }

private:
    rclcpp::Node & _node;
};
