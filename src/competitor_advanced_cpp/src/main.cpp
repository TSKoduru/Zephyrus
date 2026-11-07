/****************************************************************************
 * skybeta competitor advanced track -- entry point.
 *
 * Registers "Skybeta Goto" with PX4 as an external flight mode:
 *
 *     ros2 run competitor_advanced_cpp skybeta_goto_mode
 *     ros2 run competitor_advanced_cpp skybeta_goto_mode --ros-args \
 *         -p target_north_m:=5.0 -p target_east_m:=0.0 -p target_down_m:=-1.5
 *
 * The uXRCE-DDS agent must already be running and talking to the flight
 * controller -- registration is a live handshake with PX4, not a local
 * operation. Once it succeeds the mode shows up in QGroundControl's mode
 * list. Binding it to an RC switch position is an FC parameter change, at
 * your own risk: as shipped, nothing selects it.
 *
 * Unlike the Python track, this path is NOT behind the skybeta safety
 * envelope. See skybeta_goto_mode.hpp.
 ****************************************************************************/
#include <chrono>
#include <memory>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <px4_ros2/components/wait_for_fmu.hpp>

#include "skybeta_goto_mode.hpp"

using namespace std::chrono_literals;  // NOLINT

namespace
{
constexpr const char * kNodeName = "skybeta_goto_mode";
constexpr auto kFmuTimeout = 30s;
}  // namespace

/**
 * A plain node that owns exactly one mode and no executor.
 *
 * px4_ros2::NodeWithMode<> would do the same in one line, but doing it by
 * hand keeps the failure modes separable: waitForFMU() failing means the DDS
 * link to the flight controller is down, while doRegister() failing after a
 * successful wait means PX4 refused the registration -- typically a px4_msgs
 * generation that does not match the firmware (this fleet runs PX4 v1.16.2
 * against px4_msgs release/1.16), or no free external-mode slot.
 */
class SkybetaGotoNode : public rclcpp::Node
{
public:
  SkybetaGotoNode()
  : Node(kNodeName)
  {
    // waitForFMU spins its own WaitSet, so this is safe before rclcpp::spin.
    if (!px4_ros2::waitForFMU(*this, rclcpp::Duration(kFmuTimeout))) {
      throw std::runtime_error(
              "no VehicleStatus from the FMU within 30 s -- is MicroXRCEAgent "
              "running on the UART, and is UXRCE_DDS_CFG set to TELEM2 (102)? "
              "Check with: ros2 topic hz /fmu/out/vehicle_status_v1");
    }
    RCLCPP_INFO(get_logger(), "FMU is talking; registering the mode");

    _mode = std::make_unique<skybeta::SkybetaGotoMode>(*this);

    if (!_mode->doRegister()) {
      throw std::runtime_error(
              "PX4 refused to register '" + skybeta::kSkybetaGotoModeName +
              "'. Check that px4_msgs matches the firmware version and that "
              "PX4 has a free external mode slot.");
    }
    RCLCPP_INFO(
      get_logger(),
      "'%s' registered. Select it in QGroundControl, or bind it to an RC mode "
      "switch position. The safety pilot still arms and can take over at any "
      "time.",
      skybeta::kSkybetaGotoModeName.c_str());
  }

private:
  std::unique_ptr<skybeta::SkybetaGotoMode> _mode;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    rclcpp::spin(std::make_shared<SkybetaGotoNode>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger(kNodeName), "%s", error.what());
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
