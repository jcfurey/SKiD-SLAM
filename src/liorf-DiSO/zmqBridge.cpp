// ZeroMQ bridge for the inter-robot channel.
//
// The paper deploys over ROS plus ZeroMQ (Section VI-B). This node is that
// second half: it carries the inter-robot topics between hosts over a ZeroMQ
// PUB/SUB mesh, so the robots do not need to share a DDS domain. In the field
// that distinction matters -- DDS discovery across subnets, over a lossy
// radio, with robots joining and leaving, is the part that tends to fail.
//
// The bridge is deliberately type-agnostic. It moves serialised bytes through
// generic publishers and subscriptions, so carrying a new message type is a
// parameter change rather than a code change, and the bridge cannot fall out
// of step with a message definition.
//
// NOT COMPILED OR RUN DURING DEVELOPMENT. The transport underneath it
// (include/skid_transport.hpp) is tested over real sockets, but this file
// needs a ROS 2 installation, which this repository's development environment
// does not have. Treat it as unverified glue: build it, run two of them on one
// host first, and confirm with `ros2 topic hz` before trusting a field
// deployment.

#include <chrono>
#include <cmath>
#include <cstring>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/serialized_message.hpp>

#include "skid_transport.hpp"

namespace {

// The inter-robot topics, and the types generic pub/sub needs for them.
// Everything else a robot publishes is local and stays local.
const std::vector<std::pair<std::string, std::string>> kDefaultTopics = {
  {"/context_info", "liorf/msg/ContextInfo"},
  {"/scan_request", "liorf/msg/ScanRequest"},
  {"/scan_data", "liorf/msg/ScanData"},
  {"/loop_info_global", "liorf/msg/LoopConstraint"},
  {"/trans_odom", "nav_msgs/msg/Odometry"},
};

}  // namespace

class ZmqBridge : public rclcpp::Node {
 public:
  explicit ZmqBridge(const rclcpp::NodeOptions & options)
  : Node("liorf_zmqBridge", options) {
    const std::string robot_id = declare_and_get<std::string>(
      "robot_id", "jackal0");
    const std::string solid_topic = declare_and_get<std::string>(
      "mapfusion.interRobot.solid_topic", "solid");

    liorf::transport::Config config;
    config.robot_id = robot_id;
    config.bind_endpoint = declare_and_get<std::string>(
      "zmq.bind_endpoint", "tcp://0.0.0.0:7447");
    config.peer_endpoints = declare_and_get<std::vector<std::string>>(
      "zmq.peer_endpoints", std::vector<std::string>{});
    config.send_high_water_mark =
      declare_and_get<int>("zmq.send_high_water_mark", 100);
    config.receive_high_water_mark =
      declare_and_get<int>("zmq.receive_high_water_mark", 100);
    config.linger_ms = declare_and_get<int>("zmq.linger_ms", 0);

    const int max_payload_mib =
      declare_and_get<int>("zmq.max_payload_mib", 64);
    if (max_payload_mib < 1) {
      throw std::invalid_argument("zmq.max_payload_mib must be at least 1");
    }
    config.max_payload_bytes =
      static_cast<std::size_t>(max_payload_mib) * 1024u * 1024u;

    // Topics may be overridden wholesale; otherwise the inter-robot set is
    // used, prefixed with the configured SOLiD topic.
    auto names = declare_and_get<std::vector<std::string>>(
      "zmq.topics", std::vector<std::string>{});
    auto types = declare_and_get<std::vector<std::string>>(
      "zmq.topic_types", std::vector<std::string>{});
    if (names.empty()) {
      for (const auto & entry : kDefaultTopics) {
        names.push_back("/" + solid_topic + entry.first);
        types.push_back(entry.second);
      }
    }
    if (names.size() != types.size()) {
      throw std::invalid_argument(
        "zmq.topics and zmq.topic_types must have the same length");
    }
    config.topics = names;

    const std::string error = liorf::transport::validate(config);
    if (!error.empty()) {
      throw std::invalid_argument("invalid zmq configuration: " + error);
    }

    _transport = std::make_unique<liorf::transport::PeerTransport>(config);
    if (!_transport->error().empty()) {
      throw std::runtime_error(
        "ZeroMQ transport failed to start: " + _transport->error());
    }

    const int echo_capacity =
      declare_and_get<int>("zmq.echo_suppressor_capacity", 256);
    const double echo_window =
      declare_and_get<double>("zmq.echo_suppressor_window_s", 5.0);
    if (echo_capacity < 1 || !std::isfinite(echo_window) || echo_window <= 0.0) {
      throw std::invalid_argument(
        "zmq.echo_suppressor_capacity and _window_s must be positive");
    }
    _echo = std::make_unique<liorf::transport::EchoSuppressor>(
      static_cast<std::size_t>(echo_capacity), echo_window);

    const int qos_depth = declare_and_get<int>("zmq.qos_depth", 20);
    if (qos_depth < 1) {
      throw std::invalid_argument("zmq.qos_depth must be at least 1");
    }
    const rclcpp::QoS qos(static_cast<std::size_t>(qos_depth));

    for (std::size_t index = 0; index < names.size(); ++index) {
      const std::string & name = names[index];
      const std::string & type = types[index];

      _publishers[name] = create_generic_publisher(name, type, qos);
      _subscriptions.push_back(create_generic_subscription(
        name, type, qos,
        [this, name](std::shared_ptr<rclcpp::SerializedMessage> message) {
          forwardToPeers(name, *message);
        }));
      RCLCPP_INFO(get_logger(), "bridging %s (%s)", name.c_str(), type.c_str());
    }

    const double poll_rate_hz =
      declare_and_get<double>("zmq.poll_rate_hz", 100.0);
    if (!std::isfinite(poll_rate_hz) || poll_rate_hz <= 0.0) {
      throw std::invalid_argument("zmq.poll_rate_hz must be positive");
    }
    _poll_timer = create_wall_timer(
      std::chrono::duration<double>(1.0 / poll_rate_hz),
      std::bind(&ZmqBridge::pollPeers, this));

    _report_period_s = declare_and_get<double>("zmq.report_period_s", 30.0);
    _report_timer = create_wall_timer(
      std::chrono::duration<double>(_report_period_s),
      std::bind(&ZmqBridge::report, this));

    RCLCPP_INFO(
      get_logger(),
      "\033[1;32m----> ZeroMQ bridge started as %s on %s with %zu peer(s).\033[0m",
      robot_id.c_str(), _transport->bound_endpoint().c_str(),
      config.peer_endpoints.size());
  }

 private:
  template<typename T>
  T declare_and_get(const std::string & name, const T & default_value) {
    if (!this->has_parameter(name)) {
      this->declare_parameter<T>(name, default_value);
    }
    T value{};
    this->get_parameter(name, value);
    return value;
  }

  void forwardToPeers(const std::string & topic,
                      const rclcpp::SerializedMessage & message) {
    const auto & raw = message.get_rcl_serialized_message();
    const auto * payload = static_cast<const std::uint8_t *>(raw.buffer);
    const std::size_t size = raw.buffer_length;

    // A message this bridge injected locally must not go straight back out:
    // that is the two-way bridging loop, and it amplifies without bound.
    if (_echo->isEcho(topic, payload, size, nowSeconds())) {
      return;
    }
    if (!_transport->publish(topic, payload, size)) {
      RCLCPP_DEBUG(get_logger(), "dropped an outbound message on %s",
                   topic.c_str());
    }
  }

  void pollPeers() {
    // Zero timeout: the wall timer already sets the cadence, and blocking
    // here would stall the executor this node shares.
    for (const auto & message : _transport->receive(0)) {
      const auto publisher = _publishers.find(message.topic);
      if (publisher == _publishers.end()) {
        continue;
      }

      _echo->remember(message.topic, message.payload.data(),
                      message.payload.size(), nowSeconds());

      rclcpp::SerializedMessage serialized(message.payload.size());
      auto & raw = serialized.get_rcl_serialized_message();
      std::memcpy(raw.buffer, message.payload.data(), message.payload.size());
      raw.buffer_length = message.payload.size();
      publisher->second->publish(serialized);
    }
  }

  double nowSeconds() const { return this->now().seconds(); }

  void report() {
    const auto & statistics = _transport->statistics();
    RCLCPP_INFO(
      get_logger(),
      "zmq bridge: sent %lu msg / %.1f kiB, received %lu msg / %.1f kiB "
      "| dropped own %lu, topic mismatch %lu, oversize %lu, malformed %lu "
      "| send failures %lu | echoes suppressed %zu",
      static_cast<unsigned long>(statistics.sent),
      static_cast<double>(statistics.bytes_sent) / 1024.0,
      static_cast<unsigned long>(statistics.received),
      static_cast<double>(statistics.bytes_received) / 1024.0,
      static_cast<unsigned long>(statistics.dropped_own),
      static_cast<unsigned long>(statistics.dropped_topic_mismatch),
      static_cast<unsigned long>(statistics.dropped_oversize),
      static_cast<unsigned long>(statistics.dropped_malformed),
      static_cast<unsigned long>(statistics.send_failures),
      _echo->suppressed());
  }

  std::unique_ptr<liorf::transport::PeerTransport> _transport;
  std::unique_ptr<liorf::transport::EchoSuppressor> _echo;
  std::map<std::string, std::shared_ptr<rclcpp::GenericPublisher>> _publishers;
  std::vector<std::shared_ptr<rclcpp::GenericSubscription>> _subscriptions;
  rclcpp::TimerBase::SharedPtr _poll_timer;
  rclcpp::TimerBase::SharedPtr _report_timer;
  double _report_period_s = 30.0;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto bridge = std::make_shared<ZmqBridge>(options);
  rclcpp::spin(bridge);
  rclcpp::shutdown();
  return 0;
}
