#include "skid_transport.hpp"

#include <algorithm>
#include <sstream>
#include <utility>

#include <zmq.hpp>

namespace liorf::transport {
namespace {

bool looksLikeEndpoint(const std::string& endpoint) {
  // ZeroMQ endpoints are "<transport>://<address>". Checking the shape here
  // turns a typo into a named configuration error instead of a silent
  // never-connects.
  const std::size_t separator = endpoint.find("://");
  if (separator == std::string::npos || separator == 0) {
    return false;
  }
  if (separator + 3 >= endpoint.size()) {
    return false;
  }
  const std::string scheme = endpoint.substr(0, separator);
  return scheme == "tcp" || scheme == "ipc" || scheme == "inproc" ||
         scheme == "pgm" || scheme == "epgm";
}

}  // namespace

std::string validate(const Config& config) {
  std::ostringstream errors;
  const auto require = [&errors](bool condition, const std::string& message) {
    if (!condition) {
      errors << (errors.tellp() == std::streampos(0) ? "" : "; ") << message;
    }
  };

  require(!config.robot_id.empty(), "robot_id must not be empty");
  require(!config.bind_endpoint.empty(), "bind_endpoint must not be empty");
  if (!config.bind_endpoint.empty()) {
    require(looksLikeEndpoint(config.bind_endpoint),
            "bind_endpoint must be <transport>://<address>, got '" +
              config.bind_endpoint + "'");
  }
  for (const auto& peer : config.peer_endpoints) {
    require(looksLikeEndpoint(peer),
            "peer endpoint must be <transport>://<address>, got '" + peer + "'");
  }
  require(!config.topics.empty(), "at least one topic must be carried");
  for (const auto& topic : config.topics) {
    require(!topic.empty(), "a carried topic must not be empty");
  }
  require(config.send_high_water_mark > 0,
          "send_high_water_mark must be positive");
  require(config.receive_high_water_mark > 0,
          "receive_high_water_mark must be positive");
  require(config.max_payload_bytes > 0, "max_payload_bytes must be positive");
  require(config.linger_ms >= 0, "linger_ms must not be negative");

  return errors.str();
}

struct PeerTransport::Impl {
  Config config;
  std::string error;
  std::string bound_endpoint;
  Statistics statistics;

  std::unique_ptr<zmq::context_t> context;
  std::unique_ptr<zmq::socket_t> publisher;
  std::unique_ptr<zmq::socket_t> subscriber;
};

PeerTransport::PeerTransport(const Config& config)
  : impl_(std::make_unique<Impl>()) {
  impl_->config = config;
  impl_->error = validate(config);
  if (!impl_->error.empty()) {
    return;
  }

  try {
    impl_->context = std::make_unique<zmq::context_t>(1);

    impl_->publisher = std::make_unique<zmq::socket_t>(
      *impl_->context, zmq::socket_type::pub);
    impl_->publisher->set(zmq::sockopt::sndhwm, config.send_high_water_mark);
    impl_->publisher->set(zmq::sockopt::linger, config.linger_ms);
    impl_->publisher->bind(config.bind_endpoint);
    impl_->bound_endpoint =
      impl_->publisher->get(zmq::sockopt::last_endpoint);

    impl_->subscriber = std::make_unique<zmq::socket_t>(
      *impl_->context, zmq::socket_type::sub);
    impl_->subscriber->set(zmq::sockopt::rcvhwm,
                           config.receive_high_water_mark);
    impl_->subscriber->set(zmq::sockopt::linger, config.linger_ms);
    for (const auto& topic : config.topics) {
      impl_->subscriber->set(zmq::sockopt::subscribe, topic);
    }
    for (const auto& peer : config.peer_endpoints) {
      impl_->subscriber->connect(peer);
    }
  } catch (const zmq::error_t& failure) {
    impl_->error = std::string("ZeroMQ setup failed: ") + failure.what();
    impl_->publisher.reset();
    impl_->subscriber.reset();
    impl_->context.reset();
  }
}

PeerTransport::~PeerTransport() = default;

const std::string& PeerTransport::error() const noexcept {
  return impl_->error;
}

const Config& PeerTransport::config() const noexcept {
  return impl_->config;
}

const std::string& PeerTransport::bound_endpoint() const noexcept {
  return impl_->bound_endpoint;
}

const Statistics& PeerTransport::statistics() const noexcept {
  return impl_->statistics;
}

bool PeerTransport::publish(const std::string& topic,
                            const std::uint8_t* payload,
                            std::size_t size) {
  if (!impl_->error.empty() || !impl_->publisher) {
    return false;
  }
  if (topic.empty() || (size > 0 && payload == nullptr)) {
    ++impl_->statistics.send_failures;
    return false;
  }
  if (size > impl_->config.max_payload_bytes) {
    ++impl_->statistics.dropped_oversize;
    ++impl_->statistics.send_failures;
    return false;
  }

  // Three frames: the topic, which is what SUB filters on; the sender, so a
  // peer can drop its own traffic; then the payload.
  try {
    zmq::message_t topic_frame(topic.data(), topic.size());
    zmq::message_t sender_frame(impl_->config.robot_id.data(),
                                impl_->config.robot_id.size());
    zmq::message_t payload_frame(payload, size);

    if (!impl_->publisher->send(topic_frame, zmq::send_flags::sndmore) ||
        !impl_->publisher->send(sender_frame, zmq::send_flags::sndmore) ||
        !impl_->publisher->send(payload_frame, zmq::send_flags::none)) {
      ++impl_->statistics.send_failures;
      return false;
    }
  } catch (const zmq::error_t&) {
    ++impl_->statistics.send_failures;
    return false;
  }

  ++impl_->statistics.sent;
  impl_->statistics.bytes_sent += size;
  return true;
}

std::vector<Message> PeerTransport::receive(int timeout_ms,
                                            std::size_t max_messages) {
  std::vector<Message> messages;
  if (!impl_->error.empty() || !impl_->subscriber) {
    return messages;
  }

  const auto& topics = impl_->config.topics;
  bool first = true;

  while (messages.size() < max_messages) {
    zmq::message_t topic_frame;
    zmq::recv_result_t result;
    try {
      if (first) {
        zmq::pollitem_t item{impl_->subscriber->handle(), 0, ZMQ_POLLIN, 0};
        zmq::poll(&item, 1, std::chrono::milliseconds(std::max(0, timeout_ms)));
        if ((item.revents & ZMQ_POLLIN) == 0) {
          break;
        }
        first = false;
      }
      result = impl_->subscriber->recv(topic_frame, zmq::recv_flags::dontwait);
    } catch (const zmq::error_t&) {
      break;
    }
    if (!result) {
      break;
    }

    // A multipart message arrives whole or not at all, so the remaining
    // frames are already queued once the first has been read.
    Message message;
    message.topic = topic_frame.to_string();

    zmq::message_t sender_frame;
    zmq::message_t payload_frame;
    const bool complete =
      topic_frame.more() &&
      impl_->subscriber->recv(sender_frame, zmq::recv_flags::none) &&
      sender_frame.more() &&
      impl_->subscriber->recv(payload_frame, zmq::recv_flags::none);
    if (!complete) {
      ++impl_->statistics.dropped_malformed;
      continue;
    }

    // ZeroMQ SUB matches a subscription as a prefix, so "/a" also delivers
    // "/abc". Only an exact topic is accepted.
    if (std::find(topics.begin(), topics.end(), message.topic) ==
        topics.end()) {
      ++impl_->statistics.dropped_topic_mismatch;
      continue;
    }

    message.sender_id = sender_frame.to_string();
    if (message.sender_id == impl_->config.robot_id) {
      ++impl_->statistics.dropped_own;
      continue;
    }

    const std::size_t size = payload_frame.size();
    if (size > impl_->config.max_payload_bytes) {
      ++impl_->statistics.dropped_oversize;
      continue;
    }

    const auto* bytes = static_cast<const std::uint8_t*>(payload_frame.data());
    message.payload.assign(bytes, bytes + size);
    ++impl_->statistics.received;
    impl_->statistics.bytes_received += size;
    messages.push_back(std::move(message));
  }

  return messages;
}

namespace {

// FNV-1a. The store is a local, short-lived echo cache, not a security
// boundary, so a fast non-cryptographic digest is the right tool; the size is
// compared alongside it to make an accidental collision harmless.
std::uint64_t digestOf(const std::uint8_t* payload, std::size_t size) {
  std::uint64_t hash = 1469598103934665603ULL;
  for (std::size_t i = 0; i < size; ++i) {
    hash ^= static_cast<std::uint64_t>(payload[i]);
    hash *= 1099511628211ULL;
  }
  return hash;
}

}  // namespace

EchoSuppressor::EchoSuppressor(std::size_t capacity, double window_seconds)
  : capacity_(capacity), window_seconds_(window_seconds) {}

void EchoSuppressor::remember(const std::string& topic,
                              const std::uint8_t* payload,
                              std::size_t size,
                              double now_seconds) {
  if (capacity_ == 0 || (size > 0 && payload == nullptr)) {
    return;
  }
  expire(now_seconds);
  while (records_.size() >= capacity_) {
    records_.pop_front();
  }
  records_.push_back(
    Record{topic, digestOf(payload, size), size, now_seconds});
}

bool EchoSuppressor::isEcho(const std::string& topic,
                            const std::uint8_t* payload,
                            std::size_t size,
                            double now_seconds) {
  if (records_.empty() || (size > 0 && payload == nullptr)) {
    return false;
  }
  expire(now_seconds);

  const std::uint64_t digest = digestOf(payload, size);
  for (auto it = records_.begin(); it != records_.end(); ++it) {
    if (it->size == size && it->digest == digest && it->topic == topic) {
      records_.erase(it);
      ++suppressed_;
      return true;
    }
  }
  return false;
}

std::size_t EchoSuppressor::expire(double now_seconds) {
  std::size_t removed = 0;
  while (!records_.empty() &&
         now_seconds - records_.front().stamp > window_seconds_) {
    records_.pop_front();
    ++removed;
  }
  return removed;
}

std::size_t EchoSuppressor::size() const noexcept { return records_.size(); }

std::size_t EchoSuppressor::suppressed() const noexcept {
  return suppressed_;
}

std::size_t EchoSuppressor::capacity() const noexcept { return capacity_; }

void EchoSuppressor::clear() { records_.clear(); }

}  // namespace liorf::transport
