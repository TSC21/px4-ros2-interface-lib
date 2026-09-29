/****************************************************************************
 * Copyright (c) 2026 PX4 Development Team.
 * SPDX-License-Identifier: BSD-3-Clause
 ****************************************************************************/

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <px4_msgs/msg/mode_overlay_reply.hpp>
#include <px4_msgs/msg/mode_overlay_request.hpp>
#include <px4_msgs/msg/mode_overlay_status.hpp>
#include <px4_ros2/components/mode_overlay.hpp>
#include <px4_ros2/utils/message_version.hpp>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

namespace {

class TestOverlay : public px4_ros2::ModeOverlayBase {
 public:
  TestOverlay(rclcpp::Node& node, Settings settings) : ModeOverlayBase(node, std::move(settings))
  {
    setSkipMessageCompatibilityCheck();
  }

 protected:
  void updateSetpoint(const px4_msgs::msg::ModeOverlayInput& /*input*/, float /*dt_s*/) override
  {
    publishStop(true);
  }
};

/** FMU side of the overlay registration and status topics. */
class FakeOverlayFmu {
 public:
  FakeOverlayFmu(const std::shared_ptr<rclcpp::Node>& node, const std::string& prefix)
  {
    _reply_pub = node->create_publisher<px4_msgs::msg::ModeOverlayReply>(
        topic<px4_msgs::msg::ModeOverlayReply>(prefix, "fmu/out/mode_overlay_reply"),
        rclcpp::QoS(4).best_effort());
    _status_pub = node->create_publisher<px4_msgs::msg::ModeOverlayStatus>(
        topic<px4_msgs::msg::ModeOverlayStatus>(prefix, "fmu/out/mode_overlay_status"),
        rclcpp::QoS(1).best_effort());
    _request_sub = node->create_subscription<px4_msgs::msg::ModeOverlayRequest>(
        topic<px4_msgs::msg::ModeOverlayRequest>(prefix, "fmu/in/mode_overlay_request"),
        rclcpp::QoS(4),
        [this](px4_msgs::msg::ModeOverlayRequest::UniquePtr request) { onRequest(*request); });
    _status_timer = node->create_wall_timer(20ms, [this]() { publishStatus(); });
  }

  std::atomic<bool> parameter_enabled{true};   ///< COM_OVL_YAW
  std::atomic<bool> grant_unrequested{false};  ///< reply with a grant nobody asked for
  std::atomic<bool> publish_status{true};
  std::atomic<bool> status_for_other_session{false};

  std::vector<bool> requestedYawAuthority()
  {
    const std::lock_guard<std::mutex> lock(_mutex);
    return _requested;
  }

 private:
  template <class T>
  static std::string topic(const std::string& prefix, const std::string& name)
  {
    return prefix + name + px4_ros2::getMessageNameVersion<T>();
  }

  void onRequest(const px4_msgs::msg::ModeOverlayRequest& request)
  {
    px4_msgs::msg::ModeOverlayReply reply{};
    reply.request_id = request.request_id;
    reply.session_id = request.session_id;
    reply.result = px4_msgs::msg::ModeOverlayReply::RESULT_ACCEPTED;
    reply.applicable_modes = request.applicable_modes;
    reply.max_deviation = request.max_deviation;
    reply.yaw_authority = grant_unrequested || (request.yaw_authority && parameter_enabled);
    {
      const std::lock_guard<std::mutex> lock(_mutex);
      _requested.push_back(request.yaw_authority);
      _session = request.session_id;
      _granted = reply.yaw_authority;
    }
    _reply_pub->publish(reply);
  }

  void publishStatus()
  {
    if (!publish_status) {
      return;
    }
    px4_msgs::msg::ModeOverlayStatus status{};
    {
      const std::lock_guard<std::mutex> lock(_mutex);
      status.session_id = status_for_other_session ? _session + 1 : _session;
      status.registered = _session != 0;
      status.yaw_authority = _granted && parameter_enabled;
    }
    status.enabled = true;
    _status_pub->publish(status);
  }

  std::mutex _mutex;
  std::vector<bool> _requested;
  uint64_t _session{0};
  bool _granted{false};
  rclcpp::Publisher<px4_msgs::msg::ModeOverlayReply>::SharedPtr _reply_pub;
  rclcpp::Publisher<px4_msgs::msg::ModeOverlayStatus>::SharedPtr _status_pub;
  rclcpp::Subscription<px4_msgs::msg::ModeOverlayRequest>::SharedPtr _request_sub;
  rclcpp::TimerBase::SharedPtr _status_timer;
};

class Spinner {
 public:
  explicit Spinner(const std::shared_ptr<rclcpp::Node>& node)
  {
    _executor.add_node(node);
    _thread = std::thread([this]() { _executor.spin(); });
  }
  ~Spinner()
  {
    _executor.cancel();
    _thread.join();
  }

 private:
  rclcpp::executors::SingleThreadedExecutor _executor;
  std::thread _thread;
};

template <class Predicate>
bool eventually(Predicate predicate, std::chrono::milliseconds timeout = 3s)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(10ms);
  }
  return predicate();
}

}  // namespace

class ModeOverlayTest : public testing::Test {
 protected:
  void SetUp() override
  {
    static unsigned instance = 0;
    _prefix = "/mode_overlay_test_" + std::to_string(instance++) + "/";
    _node = std::make_shared<rclcpp::Node>("mode_overlay_test");
    _fmu_node = std::make_shared<rclcpp::Node>("mode_overlay_fmu");
    _fmu = std::make_unique<FakeOverlayFmu>(_fmu_node, _prefix);
    _fmu_spinner = std::make_unique<Spinner>(_fmu_node);
  }

  void TearDown() override
  {
    _fmu_spinner.reset();
    _fmu.reset();
  }

  px4_ros2::ModeOverlayBase::Settings settings(bool request_yaw_authority) const
  {
    px4_ros2::ModeOverlayBase::Settings settings;
    settings.topic_namespace_prefix = _prefix;
    settings.request_yaw_authority = request_yaw_authority;
    return settings;
  }

  std::string _prefix;
  std::shared_ptr<rclcpp::Node> _node;
  std::shared_ptr<rclcpp::Node> _fmu_node;
  std::unique_ptr<FakeOverlayFmu> _fmu;
  std::unique_ptr<Spinner> _fmu_spinner;
};

TEST_F(ModeOverlayTest, RequestsHeadingAuthorityOnlyWhenAsked)
{
  TestOverlay plain(*_node, settings(false));
  ASSERT_TRUE(plain.doRegister(3s));
  TestOverlay heading(*_node, settings(true));
  ASSERT_TRUE(heading.doRegister(3s));
  const auto requested = _fmu->requestedYawAuthority();
  ASSERT_GE(requested.size(), 2u);
  EXPECT_FALSE(requested.front());
  EXPECT_TRUE(requested.back());
}

TEST_F(ModeOverlayTest, ReportsHeadingAuthorityFromFreshStatus)
{
  TestOverlay overlay(*_node, settings(true));
  ASSERT_TRUE(overlay.doRegister(3s));
  EXPECT_FALSE(overlay.yawAuthority());  // no status received yet
  Spinner spinner(_node);
  EXPECT_TRUE(eventually([&]() { return overlay.yawAuthority(); }));

  // The parameter revokes the grant.
  _fmu->parameter_enabled = false;
  EXPECT_TRUE(eventually([&]() { return !overlay.yawAuthority(); }));
  _fmu->parameter_enabled = true;
  EXPECT_TRUE(eventually([&]() { return overlay.yawAuthority(); }));

  // Another session's status does not grant this one.
  _fmu->status_for_other_session = true;
  EXPECT_TRUE(eventually([&]() { return !overlay.yawAuthority(); }));
  _fmu->status_for_other_session = false;
  EXPECT_TRUE(eventually([&]() { return overlay.yawAuthority(); }));

  // A stale grant expires.
  _fmu->publish_status = false;
  EXPECT_TRUE(eventually([&]() { return !overlay.yawAuthority(); }, 2s));
}

TEST_F(ModeOverlayTest, DeniedHeadingAuthorityKeepsTheRegistration)
{
  _fmu->parameter_enabled = false;
  TestOverlay overlay(*_node, settings(true));
  ASSERT_TRUE(overlay.doRegister(3s));
  Spinner spinner(_node);
  EXPECT_TRUE(eventually([&]() { return overlay.status().registered; }));
  EXPECT_FALSE(overlay.yawAuthority());
}

TEST_F(ModeOverlayTest, RejectsAnUnrequestedHeadingGrant)
{
  _fmu->grant_unrequested = true;
  TestOverlay overlay(*_node, settings(false));
  EXPECT_FALSE(overlay.doRegister(1s));
  EXPECT_FALSE(overlay.registered());
}
