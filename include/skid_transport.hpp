#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

// Peer-to-peer transport for the inter-robot channel.
//
// The paper deploys over ROS plus ZeroMQ (Section VI-B) rather than over a
// shared DDS domain. That choice matters in the field: DDS discovery across
// subnets, over a lossy radio, with robots joining and leaving, is the part
// that tends to fail, while a ZeroMQ PUB/SUB mesh has explicit endpoints, no
// discovery, and drops rather than blocks when a peer is unreachable.
//
// This header is the transport itself and knows nothing about ROS. Message
// payloads are opaque bytes, so the bridge above it can carry any type without
// per-type code.
namespace liorf::transport {

struct Config {
  // This robot's identity, stamped on every message it sends so a peer can
  // recognise and drop its own traffic coming back.
  std::string robot_id;

  // Endpoint this robot binds its publisher to, for example
  // "tcp://0.0.0.0:7447".
  std::string bind_endpoint;

  // Endpoints of the peers to subscribe to, for example
  // {"tcp://192.168.1.12:7447"}.
  std::vector<std::string> peer_endpoints;

  // Topics carried between robots. A subscriber receives only these.
  std::vector<std::string> topics;

  // Outbound and inbound high-water marks, in messages. ZeroMQ drops PUB
  // messages beyond the send mark rather than blocking, which is the
  // behaviour a field link needs: a robot that cannot be reached must not
  // stall the robot trying to reach it.
  int send_high_water_mark = 100;
  int receive_high_water_mark = 100;

  // Bound on a single accepted payload. A frame larger than this is dropped
  // and counted rather than allocated.
  std::size_t max_payload_bytes = 64u * 1024u * 1024u;

  // Linger on shutdown, in milliseconds. Zero discards unsent messages
  // immediately, so a node cannot hang on exit waiting for an absent peer.
  int linger_ms = 0;
};

// Returns an empty string when every configuration value is usable.
std::string validate(const Config& config);

struct Message {
  std::string topic;
  std::string sender_id;
  std::vector<std::uint8_t> payload;
};

struct Statistics {
  std::uint64_t sent = 0;
  std::uint64_t send_failures = 0;
  std::uint64_t received = 0;
  // Messages this robot sent, seen coming back from the mesh.
  std::uint64_t dropped_own = 0;
  // Frames whose topic only shares a prefix with a subscription.
  std::uint64_t dropped_topic_mismatch = 0;
  std::uint64_t dropped_oversize = 0;
  std::uint64_t dropped_malformed = 0;
  std::uint64_t bytes_sent = 0;
  std::uint64_t bytes_received = 0;
};

// A ZeroMQ PUB socket bound locally and a SUB socket connected to each peer.
//
// PUB/SUB is the right shape here: fan-out is native, an unreachable peer
// cannot apply back pressure to the others, and there is no request/response
// state to recover after a link drops.
class PeerTransport {
 public:
  explicit PeerTransport(const Config& config);
  ~PeerTransport();

  PeerTransport(const PeerTransport&) = delete;
  PeerTransport& operator=(const PeerTransport&) = delete;

  // Non-empty when construction failed; such a transport neither sends nor
  // receives, so a misconfigured bridge is inert rather than half-working.
  const std::string& error() const noexcept;
  const Config& config() const noexcept;

  // Queues one message for every connected peer. Returns false when the
  // message could not be queued, which on a PUB socket means the send
  // high-water mark is reached for every peer.
  bool publish(const std::string& topic,
               const std::uint8_t* payload,
               std::size_t size);

  // Collects messages waiting on the subscriber, up to `max_messages`.
  // Waits at most `timeout_ms` for the first one, then drains without
  // blocking. Returns the messages addressed to this robot's subscriptions
  // and not sent by it.
  std::vector<Message> receive(int timeout_ms, std::size_t max_messages = 64);

  const Statistics& statistics() const noexcept;

  // The endpoint the publisher actually bound to. With a wildcard port such
  // as "tcp://127.0.0.1:*" this is the concrete endpoint, which is what a
  // test or a discovery mechanism needs.
  const std::string& bound_endpoint() const noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// Recognises a message the bridge itself injected into ROS.
//
// A topic bridged in both directions loops: the bridge publishes a peer's
// message onto the local topic, its own subscription to that topic sees it,
// and it goes straight back out with this robot as the sender. The peer then
// does the same, and the traffic amplifies without bound.
//
// Remembering what was injected, briefly, breaks that. The window only has to
// cover the round trip through the local ROS graph, so it is short and the
// store is small.
//
// Limitation: two genuinely distinct messages with byte-identical payloads on
// the same topic within the window are indistinguishable, and the second is
// suppressed. Serialised ROS messages carry a header stamp, so this does not
// arise in practice for the topics carried here, but it is a property of the
// approach rather than an implementation detail.
class EchoSuppressor {
 public:
  EchoSuppressor(std::size_t capacity, double window_seconds);

  // Records a payload this bridge is about to publish onto a local topic.
  void remember(const std::string& topic,
                const std::uint8_t* payload,
                std::size_t size,
                double now_seconds);

  // True when this payload is one that was just injected, in which case it
  // must not be forwarded back out. Consumes the record.
  bool isEcho(const std::string& topic,
              const std::uint8_t* payload,
              std::size_t size,
              double now_seconds);

  // Drops records older than the window. Called automatically by both
  // operations; exposed so a caller can bound the store while idle.
  std::size_t expire(double now_seconds);

  std::size_t size() const noexcept;
  std::size_t suppressed() const noexcept;
  std::size_t capacity() const noexcept;
  void clear();

 private:
  struct Record {
    std::string topic;
    std::uint64_t digest = 0;
    std::size_t size = 0;
    double stamp = 0.0;
  };

  std::size_t capacity_;
  double window_seconds_;
  std::size_t suppressed_ = 0;
  std::deque<Record> records_;
};

}  // namespace liorf::transport
