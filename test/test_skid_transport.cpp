#include <chrono>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "skid_transport.hpp"

namespace {

using liorf::transport::Config;
using liorf::transport::Message;
using liorf::transport::PeerTransport;

constexpr const char* kContextTopic = "/solid/context_info";
constexpr const char* kScanTopic = "/solid/scan_data";

Config baseConfig(const std::string& robot_id) {
  Config config;
  config.robot_id = robot_id;
  // Port 0 asks the OS for a free port, so parallel test runs cannot collide.
  config.bind_endpoint = "tcp://127.0.0.1:0";
  config.topics = {kContextTopic, kScanTopic};
  config.linger_ms = 0;
  return config;
}

std::vector<std::uint8_t> bytes(const std::string& text) {
  return std::vector<std::uint8_t>(text.begin(), text.end());
}

std::string text(const std::vector<std::uint8_t>& payload) {
  return std::string(payload.begin(), payload.end());
}

bool publish(PeerTransport& transport, const std::string& topic,
             const std::string& payload) {
  const auto data = bytes(payload);
  return transport.publish(topic, data.data(), data.size());
}

// PUB/SUB drops anything published before the subscriber has finished
// connecting, so a test that publishes once and reads once is inherently
// flaky. Retrying until the connection is up is the documented way to
// synchronise, and it is bounded so a genuine failure still fails.
std::vector<Message> publishUntilReceived(
  PeerTransport& sender, PeerTransport& receiver, const std::string& topic,
  const std::string& payload, int attempts = 200) {
  for (int attempt = 0; attempt < attempts; ++attempt) {
    publish(sender, topic, payload);
    auto messages = receiver.receive(25);
    if (!messages.empty()) {
      return messages;
    }
  }
  return {};
}

}  // namespace

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

TEST(SkidTransportConfig, AcceptsAUsableConfiguration) {
  EXPECT_TRUE(liorf::transport::validate(baseConfig("jackal0")).empty());
}

TEST(SkidTransportConfig, RejectsMissingIdentityAndEndpoints) {
  Config config = baseConfig("");
  EXPECT_FALSE(liorf::transport::validate(config).empty());

  config = baseConfig("jackal0");
  config.bind_endpoint = "";
  EXPECT_FALSE(liorf::transport::validate(config).empty());

  config = baseConfig("jackal0");
  config.topics.clear();
  EXPECT_FALSE(liorf::transport::validate(config).empty());
}

TEST(SkidTransportConfig, RejectsMalformedEndpoints) {
  Config config = baseConfig("jackal0");
  config.bind_endpoint = "127.0.0.1:7447";  // no transport scheme
  EXPECT_FALSE(liorf::transport::validate(config).empty());

  config = baseConfig("jackal0");
  config.bind_endpoint = "carrier-pigeon://127.0.0.1:7447";
  EXPECT_FALSE(liorf::transport::validate(config).empty());

  config = baseConfig("jackal0");
  config.peer_endpoints = {"tcp://127.0.0.1:7447", "nonsense"};
  EXPECT_FALSE(liorf::transport::validate(config).empty());
}

TEST(SkidTransportConfig, RejectsNonPositiveBounds) {
  Config config = baseConfig("jackal0");
  config.send_high_water_mark = 0;
  EXPECT_FALSE(liorf::transport::validate(config).empty());

  config = baseConfig("jackal0");
  config.max_payload_bytes = 0;
  EXPECT_FALSE(liorf::transport::validate(config).empty());

  config = baseConfig("jackal0");
  config.linger_ms = -1;
  EXPECT_FALSE(liorf::transport::validate(config).empty());
}

TEST(SkidTransport, AMisconfiguredTransportIsInertRatherThanHalfWorking) {
  Config config = baseConfig("jackal0");
  config.topics.clear();
  PeerTransport transport(config);

  ASSERT_FALSE(transport.error().empty());
  EXPECT_FALSE(publish(transport, kContextTopic, "payload"));
  EXPECT_TRUE(transport.receive(0).empty());
}

// ---------------------------------------------------------------------------
// Real sockets
// ---------------------------------------------------------------------------

TEST(SkidTransport, BindsAndReportsAConcretePort) {
  PeerTransport transport(baseConfig("jackal0"));
  ASSERT_TRUE(transport.error().empty()) << transport.error();

  const std::string endpoint = transport.bound_endpoint();
  EXPECT_EQ(0u, endpoint.rfind("tcp://127.0.0.1:", 0));
  EXPECT_EQ(std::string::npos, endpoint.find(":0"));
}

TEST(SkidTransport, CarriesAPayloadBetweenTwoPeers) {
  PeerTransport sender(baseConfig("jackal0"));
  ASSERT_TRUE(sender.error().empty()) << sender.error();

  Config receiver_config = baseConfig("jackal1");
  receiver_config.peer_endpoints = {sender.bound_endpoint()};
  PeerTransport receiver(receiver_config);
  ASSERT_TRUE(receiver.error().empty()) << receiver.error();

  const auto messages = publishUntilReceived(
    sender, receiver, kContextTopic, "descriptor bytes");
  ASSERT_FALSE(messages.empty());
  EXPECT_EQ(kContextTopic, messages[0].topic);
  EXPECT_EQ("jackal0", messages[0].sender_id);
  EXPECT_EQ("descriptor bytes", text(messages[0].payload));
  EXPECT_GT(receiver.statistics().received, 0u);
  EXPECT_GT(sender.statistics().sent, 0u);
}

TEST(SkidTransport, DeliversOnlySubscribedTopics) {
  PeerTransport sender(baseConfig("jackal0"));
  ASSERT_TRUE(sender.error().empty()) << sender.error();

  Config receiver_config = baseConfig("jackal1");
  receiver_config.peer_endpoints = {sender.bound_endpoint()};
  receiver_config.topics = {kScanTopic};  // not subscribed to context_info
  PeerTransport receiver(receiver_config);
  ASSERT_TRUE(receiver.error().empty()) << receiver.error();

  // Establish the connection with a topic the receiver does want.
  ASSERT_FALSE(publishUntilReceived(sender, receiver, kScanTopic, "scan")
                 .empty());

  for (int attempt = 0; attempt < 20; ++attempt) {
    publish(sender, kContextTopic, "descriptor");
  }
  EXPECT_TRUE(receiver.receive(50).empty());
}

TEST(SkidTransport, RejectsATopicThatOnlySharesAPrefix) {
  // ZeroMQ SUB matches subscriptions as prefixes, so a peer publishing
  // "/solid/context_info_extra" would otherwise be delivered to a subscriber
  // that asked only for "/solid/context_info".
  PeerTransport sender(baseConfig("jackal0"));
  ASSERT_TRUE(sender.error().empty()) << sender.error();

  Config receiver_config = baseConfig("jackal1");
  receiver_config.peer_endpoints = {sender.bound_endpoint()};
  receiver_config.topics = {kContextTopic};
  PeerTransport receiver(receiver_config);
  ASSERT_TRUE(receiver.error().empty()) << receiver.error();

  ASSERT_FALSE(publishUntilReceived(sender, receiver, kContextTopic, "wanted")
                 .empty());

  const std::string prefixed = std::string(kContextTopic) + "_extra";
  for (int attempt = 0; attempt < 20; ++attempt) {
    publish(sender, prefixed, "unwanted");
  }
  EXPECT_TRUE(receiver.receive(50).empty());
  EXPECT_GT(receiver.statistics().dropped_topic_mismatch, 0u);
}

TEST(SkidTransport, DropsItsOwnTrafficComingBack) {
  // Two robots in a mesh subscribe to each other; a robot must not act on the
  // messages it published itself.
  PeerTransport first(baseConfig("jackal0"));
  ASSERT_TRUE(first.error().empty()) << first.error();

  Config second_config = baseConfig("jackal1");
  second_config.peer_endpoints = {first.bound_endpoint()};
  PeerTransport second(second_config);
  ASSERT_TRUE(second.error().empty()) << second.error();

  // Now let the first subscribe to itself as well as to the second, which is
  // what a naive full-mesh configuration produces.
  Config loopback_config = baseConfig("jackal0");
  loopback_config.peer_endpoints = {first.bound_endpoint()};
  PeerTransport loopback(loopback_config);
  ASSERT_TRUE(loopback.error().empty()) << loopback.error();

  for (int attempt = 0; attempt < 100; ++attempt) {
    publish(first, kContextTopic, "mine");
    if (loopback.statistics().dropped_own > 0) {
      break;
    }
    loopback.receive(25);
  }

  EXPECT_TRUE(loopback.receive(50).empty());
  EXPECT_GT(loopback.statistics().dropped_own, 0u);
  EXPECT_EQ(0u, loopback.statistics().received);
}

TEST(SkidTransport, CarriesAnEmptyPayload) {
  PeerTransport sender(baseConfig("jackal0"));
  Config receiver_config = baseConfig("jackal1");
  receiver_config.peer_endpoints = {sender.bound_endpoint()};
  PeerTransport receiver(receiver_config);
  ASSERT_TRUE(receiver.error().empty()) << receiver.error();

  std::vector<Message> messages;
  for (int attempt = 0; attempt < 200 && messages.empty(); ++attempt) {
    sender.publish(kScanTopic, nullptr, 0);
    messages = receiver.receive(25);
  }
  ASSERT_FALSE(messages.empty());
  EXPECT_TRUE(messages[0].payload.empty());
  EXPECT_EQ(kScanTopic, messages[0].topic);
}

TEST(SkidTransport, CarriesAPayloadTheSizeOfAScan) {
  PeerTransport sender(baseConfig("jackal0"));
  Config receiver_config = baseConfig("jackal1");
  receiver_config.peer_endpoints = {sender.bound_endpoint()};
  PeerTransport receiver(receiver_config);
  ASSERT_TRUE(receiver.error().empty()) << receiver.error();

  // A feature cloud is megabytes, which is the case the framing has to hold.
  std::vector<std::uint8_t> payload(4u * 1024u * 1024u);
  for (std::size_t i = 0; i < payload.size(); ++i) {
    payload[i] = static_cast<std::uint8_t>(i & 0xFF);
  }

  std::vector<Message> messages;
  for (int attempt = 0; attempt < 200 && messages.empty(); ++attempt) {
    sender.publish(kScanTopic, payload.data(), payload.size());
    messages = receiver.receive(50);
  }
  ASSERT_FALSE(messages.empty());
  EXPECT_EQ(payload.size(), messages[0].payload.size());
  EXPECT_EQ(payload, messages[0].payload);
}

TEST(SkidTransport, RefusesAPayloadBeyondTheConfiguredBound) {
  Config config = baseConfig("jackal0");
  config.max_payload_bytes = 1024;
  PeerTransport transport(config);
  ASSERT_TRUE(transport.error().empty()) << transport.error();

  const std::vector<std::uint8_t> payload(2048, 0x7F);
  EXPECT_FALSE(
    transport.publish(kScanTopic, payload.data(), payload.size()));
  EXPECT_EQ(1u, transport.statistics().dropped_oversize);
  EXPECT_EQ(0u, transport.statistics().sent);
}

TEST(SkidTransport, RejectsAnEmptyTopicAndANullPayload) {
  PeerTransport transport(baseConfig("jackal0"));
  ASSERT_TRUE(transport.error().empty()) << transport.error();

  EXPECT_FALSE(publish(transport, "", "payload"));
  EXPECT_FALSE(transport.publish(kScanTopic, nullptr, 16));
  EXPECT_EQ(2u, transport.statistics().send_failures);
}

TEST(SkidTransport, PublishingWithNoPeerNeitherBlocksNorFails) {
  // The field case: a robot keeps announcing while nobody is listening. PUB
  // discards, so the call must return promptly and report success.
  PeerTransport transport(baseConfig("jackal0"));
  ASSERT_TRUE(transport.error().empty()) << transport.error();

  const auto started = std::chrono::steady_clock::now();
  for (int i = 0; i < 500; ++i) {
    EXPECT_TRUE(publish(transport, kContextTopic, "descriptor"));
  }
  const auto elapsed = std::chrono::steady_clock::now() - started;

  EXPECT_LT(std::chrono::duration_cast<std::chrono::seconds>(elapsed).count(),
            5);
  EXPECT_EQ(500u, transport.statistics().sent);
}

TEST(SkidTransport, ReceiveReturnsPromptlyWhenNothingArrives) {
  PeerTransport transport(baseConfig("jackal0"));
  ASSERT_TRUE(transport.error().empty()) << transport.error();

  const auto started = std::chrono::steady_clock::now();
  EXPECT_TRUE(transport.receive(20).empty());
  const auto elapsed = std::chrono::steady_clock::now() - started;
  EXPECT_LT(
    std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(),
    2000);
}

TEST(SkidTransport, DrainsSeveralMessagesInOneCall) {
  PeerTransport sender(baseConfig("jackal0"));
  Config receiver_config = baseConfig("jackal1");
  receiver_config.peer_endpoints = {sender.bound_endpoint()};
  PeerTransport receiver(receiver_config);
  ASSERT_TRUE(receiver.error().empty()) << receiver.error();

  ASSERT_FALSE(publishUntilReceived(sender, receiver, kContextTopic, "warmup")
                 .empty());

  for (int i = 0; i < 5; ++i) {
    ASSERT_TRUE(publish(sender, kContextTopic, "batch " + std::to_string(i)));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  const auto messages = receiver.receive(100, 16);
  EXPECT_GE(messages.size(), 2u);
  for (const auto& message : messages) {
    EXPECT_EQ(kContextTopic, message.topic);
    EXPECT_EQ("jackal0", message.sender_id);
  }
}

TEST(SkidTransport, HonoursTheRequestedBatchLimit) {
  PeerTransport sender(baseConfig("jackal0"));
  Config receiver_config = baseConfig("jackal1");
  receiver_config.peer_endpoints = {sender.bound_endpoint()};
  PeerTransport receiver(receiver_config);
  ASSERT_TRUE(receiver.error().empty()) << receiver.error();

  ASSERT_FALSE(publishUntilReceived(sender, receiver, kContextTopic, "warmup")
                 .empty());

  for (int i = 0; i < 8; ++i) {
    publish(sender, kContextTopic, "batch");
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  EXPECT_LE(receiver.receive(100, 2).size(), 2u);
}

// ---------------------------------------------------------------------------
// Echo suppression
// ---------------------------------------------------------------------------

namespace {

using liorf::transport::EchoSuppressor;

void remember(EchoSuppressor& suppressor, const std::string& topic,
              const std::string& payload, double now) {
  const auto data = bytes(payload);
  suppressor.remember(topic, data.data(), data.size(), now);
}

bool isEcho(EchoSuppressor& suppressor, const std::string& topic,
            const std::string& payload, double now) {
  const auto data = bytes(payload);
  return suppressor.isEcho(topic, data.data(), data.size(), now);
}

}  // namespace

TEST(SkidEchoSuppressor, RecognisesWhatWasJustInjected) {
  EchoSuppressor suppressor(64, 5.0);
  remember(suppressor, kContextTopic, "payload", 100.0);

  EXPECT_TRUE(isEcho(suppressor, kContextTopic, "payload", 100.1));
  EXPECT_EQ(1u, suppressor.suppressed());
}

TEST(SkidEchoSuppressor, SuppressesEachInjectionOnlyOnce) {
  EchoSuppressor suppressor(64, 5.0);
  remember(suppressor, kContextTopic, "payload", 100.0);

  ASSERT_TRUE(isEcho(suppressor, kContextTopic, "payload", 100.1));
  // The record is consumed, so a genuinely new message with the same bytes
  // still gets through.
  EXPECT_FALSE(isEcho(suppressor, kContextTopic, "payload", 100.2));
}

TEST(SkidEchoSuppressor, DistinguishesTopicAndPayload) {
  EchoSuppressor suppressor(64, 5.0);
  remember(suppressor, kContextTopic, "payload", 100.0);

  EXPECT_FALSE(isEcho(suppressor, kScanTopic, "payload", 100.1));
  EXPECT_FALSE(isEcho(suppressor, kContextTopic, "different", 100.1));
  EXPECT_TRUE(isEcho(suppressor, kContextTopic, "payload", 100.1));
}

TEST(SkidEchoSuppressor, ForgetsBeyondTheWindow) {
  EchoSuppressor suppressor(64, 5.0);
  remember(suppressor, kContextTopic, "payload", 100.0);

  EXPECT_FALSE(isEcho(suppressor, kContextTopic, "payload", 106.0));
  EXPECT_EQ(0u, suppressor.size());
}

TEST(SkidEchoSuppressor, StaysWithinItsCapacity) {
  EchoSuppressor suppressor(4, 60.0);
  for (int i = 0; i < 10; ++i) {
    remember(suppressor, kContextTopic, "payload " + std::to_string(i), 100.0);
  }
  EXPECT_LE(suppressor.size(), 4u);
  // The oldest were dropped to make room, so they are no longer recognised.
  EXPECT_FALSE(isEcho(suppressor, kContextTopic, "payload 0", 100.0));
  EXPECT_TRUE(isEcho(suppressor, kContextTopic, "payload 9", 100.0));
}

TEST(SkidEchoSuppressor, HandlesEmptyPayloadsAndAZeroCapacity) {
  EchoSuppressor suppressor(8, 5.0);
  suppressor.remember(kScanTopic, nullptr, 0, 100.0);
  EXPECT_TRUE(suppressor.isEcho(kScanTopic, nullptr, 0, 100.1));

  EchoSuppressor disabled(0, 5.0);
  remember(disabled, kContextTopic, "payload", 100.0);
  EXPECT_EQ(0u, disabled.size());
  EXPECT_FALSE(isEcho(disabled, kContextTopic, "payload", 100.0));
}

TEST(SkidEchoSuppressor, BreaksATwoWayBridgeLoop) {
  // The loop this exists to stop: a peer's message is injected locally, the
  // bridge's own subscription sees it, and without suppression it goes back
  // out and amplifies.
  EchoSuppressor suppressor(64, 5.0);
  const std::string payload = "descriptor from jackal1";

  int forwarded = 0;
  for (int round = 0; round < 10; ++round) {
    // Inbound from a peer: record it, then publish onto the local topic.
    remember(suppressor, kContextTopic, payload, 100.0 + round);
    // The local subscription immediately sees exactly those bytes.
    if (!isEcho(suppressor, kContextTopic, payload, 100.0 + round)) {
      ++forwarded;
    }
  }
  EXPECT_EQ(0, forwarded);
  EXPECT_EQ(10u, suppressor.suppressed());
}
