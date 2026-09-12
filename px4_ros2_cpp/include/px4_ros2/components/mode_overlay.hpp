/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#pragma once

#include <chrono>
#include <memory>
#include <px4_msgs/msg/mode_overlay_input.hpp>
#include <px4_msgs/msg/mode_overlay_status.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace px4_ros2 {

/**
 * A registered companion overlay of multicopter position-control intent.
 *
 * Requires PX4's explicit mode-overlay capability. Existing modes retain ownership
 * of their intent. Registration never arms or changes a mode. The FMU owns
 * arbitration, freshness, authority bounds, arming checks and the loss response.
 * Call doRegister() before spinning; use one executor callback at a time per node.
 */
class ModeOverlayBase {
 public:
  struct Settings {
    std::string name{"Navigation Overlay"};
    uint32_t applicable_modes_mask{0x7f80407cU};
    float max_position_deviation_m{3.f};
    std::string topic_namespace_prefix{};
  };

  ModeOverlayBase(rclcpp::Node& node, Settings settings);
  virtual ~ModeOverlayBase();
  ModeOverlayBase(const ModeOverlayBase&) = delete;
  ModeOverlayBase& operator=(const ModeOverlayBase&) = delete;
  ModeOverlayBase(ModeOverlayBase&&) = delete;
  ModeOverlayBase& operator=(ModeOverlayBase&&) = delete;

  /** Check message compatibility and complete a real FMU registration handshake. */
  bool doRegister(std::chrono::milliseconds timeout = std::chrono::seconds(5));
  bool registered() const;
  bool engaged() const;
  const px4_msgs::msg::ModeOverlayStatus& status() const;

 protected:
  /** Fast callback for each new FMU intent. Publish exactly one response here. */
  virtual void updateSetpoint(const px4_msgs::msg::ModeOverlayInput& input, float dt_s) = 0;
  virtual void onEngaged() {}
  virtual void onDisengaged() {}

  /** Full finite local-NED p/v/a/jerk; the FMU preserves the source mode's yaw. */
  bool publishReplacement(const px4_msgs::msg::TrajectorySetpoint& setpoint, bool ready = true);
  bool publishPassthrough(bool ready = true);
  /** STOP invokes PX4's jerk-limited brake. A healthy planner can legitimately stop. */
  bool publishStop(bool ready);

 private:
  class Impl;
  std::unique_ptr<Impl> _impl;
};

}  // namespace px4_ros2
