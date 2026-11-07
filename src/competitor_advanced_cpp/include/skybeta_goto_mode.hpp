/****************************************************************************
 * skybeta competitor advanced track -- a PX4 custom flight mode.
 *
 * Modelled on px4-ros2-interface-lib release/1.16
 * examples/cpp/modes/goto (BSD-3-Clause, PX4 Development Team).
 *
 * WHAT THIS IS, AND WHY IT IS THE *ADVANCED* TRACK
 * ------------------------------------------------
 * The Python track (skybeta_flight_interface + competitor_example) drives
 * PX4's built-in Offboard mode: your node streams setpoints, the safety pilot
 * arms, then switches to Position, and the flight interface enters Offboard and clamps
 * everything you ask for to the safety envelope first.
 *
 * This is different. px4_ros2_cpp registers a *new flight mode* with PX4 over
 * uXRCE-DDS. Once registered it appears in QGroundControl's mode list and can
 * be bound to an RC mode switch position exactly like Position or Altitude,
 * and PX4's own commander runs its arming checks against it. As shipped nothing
 * selects it: the fleet transmitter's switch holds Manual, Altitude and Position,
 * and binding a mode is an FC parameter change, at your own risk. That is a nicer
 * fit for a competition -- the pilot selects your mode by name -- but it
 * comes with real extra risk:
 *
 *   * There is NO geofence, altitude clamp or speed cap in this path. The
 *     Python flight interface is not in the loop at all. Whatever
 *     updateSetpoint() publishes goes to the controller. You own the safety
 *     envelope yourself; the constants below are a starting point, not a
 *     guarantee.
 *   * Setpoints here are NED, not ENU. Positive z is DOWN. A sign error puts
 *     the vehicle in the ground.
 *   * The mode is fed by PX4's own local-position estimate
 *     (OdometryLocalPosition), which is EKF2's fused output -- not the raw
 *     cuVSLAM odometry. GPS-denied, that estimate is only as good as the
 *     vision link feeding it, so an intermittent EV datalink degrades this
 *     mode silently.
 *   * Registration is a runtime handshake. If px4_msgs does not match the
 *     firmware, or the DDS link is down, the node throws at construction.
 *
 * The mode flies to one parameterised NED target and reports completion.
 * Keep it that simple until you have flown it.
 ****************************************************************************/
#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/goto.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/utils/geometry.hpp>

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>

#include <algorithm>
#include <string>

namespace skybeta
{

/// Shown in QGroundControl's mode list. PX4 requires < 25 characters.
static const std::string kSkybetaGotoModeName = "Skybeta Goto";

class SkybetaGotoMode : public px4_ros2::ModeBase
{
public:
  explicit SkybetaGotoMode(rclcpp::Node & node)
  : ModeBase(node, kSkybetaGotoModeName),
    _node(node)
  {
    // Target in PX4's NED local frame. z is DOWN, so the default -1.5 is
    // 1.5 m ABOVE the EKF2 origin. Declared on the node, so they can be set
    // from the command line or a launch file:
    //   ros2 run competitor_advanced_cpp skybeta_goto_mode --ros-args
    //        -p target_north_m:=5.0 -p target_down_m:=-2.0
    _target_north_m = declare<double>("target_north_m", 5.0);
    _target_east_m = declare<double>("target_east_m", 0.0);
    _target_down_m = declare<double>("target_down_m", -1.5);
    _target_yaw_deg = declare<double>("target_yaw_deg", NAN);
    _max_horizontal_speed_m_s = declare<double>("max_horizontal_speed_m_s", 1.5);
    _max_vertical_speed_m_s = declare<double>("max_vertical_speed_m_s", 0.8);
    _position_threshold_m = declare<double>("position_threshold_m", 0.3);
    _velocity_threshold_m_s = declare<double>("velocity_threshold_m_s", 0.3);
    // Relative to the position the vehicle holds when the mode is activated,
    // rather than to the EKF2 origin, which may be anywhere.
    _relative_to_activation = declare<bool>("relative_to_activation", true);

    _goto_setpoint = std::make_shared<px4_ros2::GotoSetpointType>(*this);
    _vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

    RCLCPP_INFO(
      _node.get_logger(),
      "'%s': target NED (%.2f, %.2f, %.2f) m %s, max %.2f m/s horizontal / "
      "%.2f m/s vertical. NOTE: the skybeta safety envelope does NOT apply to "
      "this path.",
      kSkybetaGotoModeName.c_str(), _target_north_m, _target_east_m, _target_down_m,
      _relative_to_activation ? "relative to the activation point" : "from the EKF2 origin",
      _max_horizontal_speed_m_s, _max_vertical_speed_m_s);
  }

  void onActivate() override
  {
    _state = State::Flying;
    _origin_set = false;
    RCLCPP_INFO(_node.get_logger(), "'%s' activated", kSkybetaGotoModeName.c_str());
  }

  void onDeactivate() override
  {
    // The pilot moved the mode switch, or PX4 took the mode off us. Nothing
    // to unwind: the next activation re-captures the origin.
    RCLCPP_INFO(_node.get_logger(), "'%s' deactivated", kSkybetaGotoModeName.c_str());
  }

  void updateSetpoint(float dt_s) override
  {
    (void)dt_s;

    if (!_origin_set) {
      // Capture once per activation, so the target is anchored to where the
      // pilot handed the vehicle over rather than to wherever EKF2 started.
      _origin_ned_m = _relative_to_activation ?
        _vehicle_local_position->positionNed() : Eigen::Vector3f::Zero();
      _origin_set = true;
      _target_ned_m = _origin_ned_m + Eigen::Vector3f{
        static_cast<float>(_target_north_m),
        static_cast<float>(_target_east_m),
        static_cast<float>(_target_down_m)};
      RCLCPP_INFO(
        _node.get_logger(), "flying to NED (%.2f, %.2f, %.2f)",
        _target_ned_m.x(), _target_ned_m.y(), _target_ned_m.z());
    }

    if (_state == State::Done) {
      // Hold the target after completion. PX4 has already been told the mode
      // finished; continuing to publish keeps the vehicle where it is until
      // the pilot selects another mode.
      _goto_setpoint->update(_target_ned_m);
      return;
    }

    const std::optional<float> heading = std::isfinite(_target_yaw_deg) ?
      std::optional<float>{px4_ros2::degToRad(static_cast<float>(_target_yaw_deg))} :
      std::nullopt;

    _goto_setpoint->update(
      _target_ned_m,
      heading,
      static_cast<float>(_max_horizontal_speed_m_s),
      static_cast<float>(_max_vertical_speed_m_s));

    if (positionReached(_target_ned_m)) {
      _state = State::Done;
      RCLCPP_INFO(_node.get_logger(), "target reached -- completing");
      completed(px4_ros2::Result::Success);
    }
  }

private:
  enum class State
  {
    Flying = 0,
    Done
  } _state{State::Flying};

  template<typename T>
  T declare(const std::string & name, T default_value)
  {
    // The mode may be constructed more than once in a process (or a
    // parameter may already exist from a launch file), so tolerate a
    // redeclaration instead of throwing.
    if (!_node.has_parameter(name)) {
      _node.declare_parameter<T>(name, default_value);
    }
    return _node.get_parameter(name).get_value<T>();
  }

  bool positionReached(const Eigen::Vector3f & target_ned_m) const
  {
    const Eigen::Vector3f error_m = target_ned_m - _vehicle_local_position->positionNed();
    return error_m.norm() < static_cast<float>(_position_threshold_m) &&
           _vehicle_local_position->velocityNed().norm() <
           static_cast<float>(_velocity_threshold_m_s);
  }

  rclcpp::Node & _node;

  double _target_north_m{};
  double _target_east_m{};
  double _target_down_m{};
  double _target_yaw_deg{};
  double _max_horizontal_speed_m_s{};
  double _max_vertical_speed_m_s{};
  double _position_threshold_m{};
  double _velocity_threshold_m_s{};
  bool _relative_to_activation{true};

  bool _origin_set{false};
  Eigen::Vector3f _origin_ned_m{Eigen::Vector3f::Zero()};
  Eigen::Vector3f _target_ned_m{Eigen::Vector3f::Zero()};

  std::shared_ptr<px4_ros2::GotoSetpointType> _goto_setpoint;
  std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
};

}  // namespace skybeta
