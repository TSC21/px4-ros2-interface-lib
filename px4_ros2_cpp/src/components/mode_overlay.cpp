/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/
#include "px4_ros2/components/mode_overlay.hpp"

#include <algorithm>
#include <cmath>
#include <px4_msgs/msg/mode_overlay_output.hpp>
#include <px4_msgs/msg/mode_overlay_reply.hpp>
#include <px4_msgs/msg/mode_overlay_request.hpp>
#include <random>
#include <stdexcept>

#include "px4_ros2/components/message_compatibility_check.hpp"
#include "px4_ros2/utils/message_version.hpp"

namespace px4_ros2 {

class ModeOverlayBase::Impl {
 public:
  using Clock = std::chrono::steady_clock;
  using Input = px4_msgs::msg::ModeOverlayInput;
  using Output = px4_msgs::msg::ModeOverlayOutput;
  using Request = px4_msgs::msg::ModeOverlayRequest;
  using Reply = px4_msgs::msg::ModeOverlayReply;
  using Status = px4_msgs::msg::ModeOverlayStatus;

  Impl(ModeOverlayBase& owner, rclcpp::Node& node, Settings settings)
      : owner(owner), node(node), settings(std::move(settings))
  {
    if (this->settings.name.empty() || this->settings.name.size() >= Request{}.name.size() ||
        this->settings.applicable_modes_mask == 0 ||
        !std::isfinite(this->settings.max_position_deviation_m) ||
        this->settings.max_position_deviation_m <= 0.f) {
      throw std::invalid_argument("Invalid mode-overlay settings");
    }
    prefix = this->settings.topic_namespace_prefix;
    if (!prefix.empty() && prefix.back() != '/') prefix += '/';
    std::random_device random;
    session = (static_cast<uint64_t>(random()) << 32U) | random();
    if (session == 0) session = 1;

    reply_group = node.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    rclcpp::SubscriptionOptions options;
    options.callback_group = reply_group;
    reply_sub = node.create_subscription<Reply>(
        topic<Reply>("out/mode_overlay_reply"), rclcpp::QoS(4).best_effort(),
        [](Reply::ConstSharedPtr) {}, options);
    request_pub = node.create_publisher<Request>(topic<Request>("in/mode_overlay_request"), 4);
    output_pub = node.create_publisher<Output>(topic<Output>("in/mode_overlay_output"), 1);
    input_sub = node.create_subscription<Input>(
        topic<Input>("out/mode_overlay_input"), rclcpp::SensorDataQoS().keep_last(1),
        [this](Input::ConstSharedPtr input) {
          if (!is_registered || input->session_id != session || input->intent_id <= last_intent) {
            return;
          }
          const auto now = Clock::now();
          const float dt = last_input_time == Clock::time_point{}
                               ? 0.02f
                               : std::chrono::duration<float>(now - last_input_time).count();
          last_input_time = now;
          last_intent = input->intent_id;
          can_publish = true;
          this->owner.updateSetpoint(*input, std::clamp(dt, 0.001f, 1.f));
          if (can_publish) publish(Output::ACTION_STOP, px4_msgs::msg::TrajectorySetpoint{}, false);
          can_publish = false;
        });
    status_sub = node.create_subscription<Status>(
        topic<Status>("out/mode_overlay_status"), rclcpp::SensorDataQoS().keep_last(1),
        [this](Status::ConstSharedPtr status) {
          last_status = *status;
          last_status_time = Clock::now();
          const bool selected = is_registered && status->session_id == session && status->engaged;
          setEngaged(selected);
        });
    watchdog = node.create_wall_timer(std::chrono::milliseconds(100), [this] {
      if (Clock::now() - last_status_time > std::chrono::milliseconds(500)) setEngaged(false);
    });
  }

  ~Impl()
  {
    if (is_registered && rclcpp::ok(node.get_node_base_interface()->get_context())) {
      auto request = registrationRequest();
      request.action = Request::ACTION_UNREGISTER;
      request_pub->publish(request);  // PX4 deliberately rejects unregister while armed.
    }
  }

  template <class T>
  std::string topic(const std::string& name) const
  {
    return prefix + "fmu/" + name + getMessageNameVersion<T>();
  }

  Request registrationRequest() const
  {
    Request request{};
    request.request_id = session;
    request.session_id = session;
    request.action = Request::ACTION_REGISTER;
    std::copy(settings.name.begin(), settings.name.end(), request.name.begin());
    request.applicable_modes = settings.applicable_modes_mask;
    request.max_deviation = settings.max_position_deviation_m;
    return request;
  }

  bool doRegister(std::chrono::milliseconds timeout)
  {
    if (is_registered) return true;
    if (timeout <= std::chrono::milliseconds::zero()) return false;
    const std::vector<MessageCompatibilityTopic> messages{{"fmu/in/mode_overlay_request"},
                                                          {"fmu/in/mode_overlay_output"},
                                                          {"fmu/out/mode_overlay_reply"},
                                                          {"fmu/out/mode_overlay_input"},
                                                          {"fmu/out/mode_overlay_status"}};
    if (!messageCompatibilityCheck(node, messages, prefix)) return false;
    rclcpp::WaitSet wait_set;
    wait_set.add_subscription(reply_sub);
    const auto deadline = Clock::now() + timeout;
    const auto request = registrationRequest();
    while (rclcpp::ok(node.get_node_base_interface()->get_context()) && Clock::now() < deadline) {
      request_pub->publish(request);
      const auto remaining = deadline - Clock::now();
      const auto retry = std::min(remaining, Clock::duration(std::chrono::milliseconds(250)));
      if (retry <= Clock::duration::zero()) break;
      if (wait_set.wait(retry).kind() != rclcpp::WaitResultKind::Ready) continue;
      Reply reply;
      rclcpp::MessageInfo info;
      if (!reply_sub->take(reply, info) || reply.request_id != request.request_id ||
          reply.session_id != session)
        continue;
      if (reply.result != Reply::RESULT_ACCEPTED) {
        RCLCPP_ERROR(node.get_logger(), "Overlay registration rejected by PX4 (reason %u)",
                     static_cast<unsigned>(reply.result));
        break;
      }
      if (reply.applicable_modes == 0 || !std::isfinite(reply.max_deviation) ||
          reply.max_deviation <= 0.f ||
          (reply.applicable_modes & ~settings.applicable_modes_mask) != 0)
        break;
      is_registered = true;
      RCLCPP_INFO(node.get_logger(), "Registered overlay '%s', granted reference radius %.2f m",
                  settings.name.c_str(), static_cast<double>(reply.max_deviation));
      break;
    }
    wait_set.remove_subscription(reply_sub);
    return is_registered;
  }

  bool publish(uint8_t action, const px4_msgs::msg::TrajectorySetpoint& setpoint, bool ready)
  {
    if (!is_registered || !can_publish) return false;
    Output output{};
    output.session_id = session;
    output.intent_id = last_intent;
    output.sequence = ++sequence;
    output.action = action;
    output.ready = ready;
    output.setpoint = setpoint;
    output.setpoint.timestamp = 0;
    output_pub->publish(output);
    can_publish = false;
    return true;
  }

  void setEngaged(bool value)
  {
    if (is_engaged == value) return;
    is_engaged = value;
    if (value)
      owner.onEngaged();
    else
      owner.onDisengaged();
  }

  ModeOverlayBase& owner;
  rclcpp::Node& node;
  Settings settings;
  std::string prefix;
  uint64_t session{0}, sequence{0}, last_intent{0};
  bool is_registered{false}, is_engaged{false}, can_publish{false};
  Status last_status{};
  Clock::time_point last_input_time{}, last_status_time{};
  rclcpp::CallbackGroup::SharedPtr reply_group;
  rclcpp::Subscription<Reply>::SharedPtr reply_sub;
  rclcpp::Subscription<Input>::SharedPtr input_sub;
  rclcpp::Subscription<Status>::SharedPtr status_sub;
  rclcpp::Publisher<Request>::SharedPtr request_pub;
  rclcpp::Publisher<Output>::SharedPtr output_pub;
  rclcpp::TimerBase::SharedPtr watchdog;
};

ModeOverlayBase::ModeOverlayBase(rclcpp::Node& node, Settings settings)
    : _impl(std::make_unique<Impl>(*this, node, std::move(settings)))
{
}
ModeOverlayBase::~ModeOverlayBase() = default;
bool ModeOverlayBase::doRegister(std::chrono::milliseconds timeout)
{
  return _impl->doRegister(timeout);
}
bool ModeOverlayBase::registered() const
{
  return _impl->is_registered;
}
bool ModeOverlayBase::engaged() const
{
  return _impl->is_engaged;
}
const px4_msgs::msg::ModeOverlayStatus& ModeOverlayBase::status() const
{
  return _impl->last_status;
}
bool ModeOverlayBase::publishReplacement(const px4_msgs::msg::TrajectorySetpoint& setpoint,
                                         bool ready)
{
  return _impl->publish(Impl::Output::ACTION_REPLACE, setpoint, ready);
}
bool ModeOverlayBase::publishPassthrough(bool ready)
{
  return _impl->publish(Impl::Output::ACTION_PASSTHROUGH, px4_msgs::msg::TrajectorySetpoint{},
                        ready);
}
bool ModeOverlayBase::publishStop(bool ready)
{
  return _impl->publish(Impl::Output::ACTION_STOP, px4_msgs::msg::TrajectorySetpoint{}, ready);
}

}  // namespace px4_ros2
