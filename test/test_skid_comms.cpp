#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "skid_comms.hpp"

namespace {

using liorf::comms::BoundedQueue;
using liorf::comms::Config;
using liorf::comms::DeferredCandidate;
using liorf::comms::DeferredCandidateQueue;
using liorf::comms::RequestDecision;
using liorf::comms::RequestTracker;
using liorf::comms::ScanCache;
using liorf::comms::ScanKey;
using liorf::comms::TransferStats;

Config testConfig() {
  Config config;
  config.max_pending_announcements = 3;
  config.max_cached_scans = 3;
  config.max_cached_scan_bytes = 1000;
  config.max_inflight_requests = 2;
  config.request_timeout_s = 5.0;
  config.max_request_attempts = 2;
  config.max_deferred_candidates = 3;
  config.max_deferred_age_s = 60.0;
  return config;
}

ScanKey key(const char* robot, std::int64_t index) {
  ScanKey scan_key;
  scan_key.robot_id = robot;
  scan_key.keyframe_index = index;
  return scan_key;
}

}  // namespace

// ---------------------------------------------------------------------------
// Configuration and keys
// ---------------------------------------------------------------------------

TEST(SkidCommsConfig, RejectsInvalidValues) {
  EXPECT_TRUE(liorf::comms::validate(testConfig()).empty());

  Config config = testConfig();
  config.max_cached_scan_bytes = 0;
  EXPECT_FALSE(liorf::comms::validate(config).empty());

  config = testConfig();
  config.request_timeout_s = 0.0;
  EXPECT_FALSE(liorf::comms::validate(config).empty());

  config = testConfig();
  config.max_deferred_age_s = std::nan("");
  EXPECT_FALSE(liorf::comms::validate(config).empty());

  config = testConfig();
  config.max_request_attempts = 0;
  EXPECT_FALSE(liorf::comms::validate(config).empty());
}

TEST(SkidCommsScanKey, RejectsIncompleteIdentities) {
  EXPECT_TRUE(key("jackal1", 0).valid());
  EXPECT_FALSE(key("", 4).valid());
  EXPECT_FALSE(key("jackal1", -1).valid());
  EXPECT_EQ("jackal1#7", key("jackal1", 7).str());

  EXPECT_EQ(key("jackal1", 7), key("jackal1", 7));
  EXPECT_NE(key("jackal1", 7), key("jackal2", 7));
  EXPECT_NE(key("jackal1", 7), key("jackal1", 8));
}

// ---------------------------------------------------------------------------
// BoundedQueue
// ---------------------------------------------------------------------------

TEST(SkidCommsBoundedQueue, DropsTheOldestWhenFull) {
  BoundedQueue<int> queue(3);
  for (int i = 0; i < 3; ++i) {
    EXPECT_FALSE(queue.push(i));
  }
  ASSERT_EQ(3u, queue.size());
  EXPECT_EQ(0u, queue.dropped());

  // The fourth push displaces the oldest, keeping the freshest places.
  EXPECT_TRUE(queue.push(3));
  EXPECT_EQ(3u, queue.size());
  EXPECT_EQ(1u, queue.dropped());

  int value = -1;
  ASSERT_TRUE(queue.pop(value));
  EXPECT_EQ(1, value);
  ASSERT_TRUE(queue.pop(value));
  EXPECT_EQ(2, value);
  ASSERT_TRUE(queue.pop(value));
  EXPECT_EQ(3, value);
  EXPECT_FALSE(queue.pop(value));
  EXPECT_TRUE(queue.empty());
}

TEST(SkidCommsBoundedQueue, ZeroCapacityDropsEverything) {
  BoundedQueue<int> queue(0);
  EXPECT_TRUE(queue.push(1));
  EXPECT_TRUE(queue.empty());
  EXPECT_EQ(1u, queue.dropped());
}

// ---------------------------------------------------------------------------
// ScanCache
// ---------------------------------------------------------------------------

TEST(SkidCommsScanCache, EvictsTheLeastRecentlyUsedOnCountOverflow) {
  ScanCache cache(testConfig());
  ASSERT_TRUE(cache.error().empty());

  EXPECT_TRUE(cache.insert(key("a", 0), 10).empty());
  EXPECT_TRUE(cache.insert(key("a", 1), 10).empty());
  EXPECT_TRUE(cache.insert(key("a", 2), 10).empty());
  ASSERT_EQ(3u, cache.size());
  EXPECT_EQ(30u, cache.bytes());

  // Using entry 0 again makes entry 1 the least recently used.
  ASSERT_TRUE(cache.touch(key("a", 0)));
  const std::vector<ScanKey> evicted = cache.insert(key("a", 3), 10);
  ASSERT_EQ(1u, evicted.size());
  EXPECT_EQ(key("a", 1), evicted[0]);
  EXPECT_FALSE(cache.contains(key("a", 1)));
  EXPECT_TRUE(cache.contains(key("a", 0)));
  EXPECT_EQ(3u, cache.size());
  EXPECT_EQ(1u, cache.evicted());
}

TEST(SkidCommsScanCache, EvictsTheFewestEntriesTheByteBudgetNeeds) {
  ScanCache cache(testConfig());  // 1000-byte budget
  cache.insert(key("a", 0), 400);
  cache.insert(key("a", 1), 400);
  ASSERT_EQ(800u, cache.bytes());

  // 400 + 400 + 500 exceeds the budget, but dropping only the oldest brings
  // it back under. The second-oldest must survive.
  const std::vector<ScanKey> evicted = cache.insert(key("a", 2), 500);
  ASSERT_EQ(1u, evicted.size());
  EXPECT_EQ(key("a", 0), evicted[0]);
  EXPECT_TRUE(cache.contains(key("a", 1)));
  EXPECT_TRUE(cache.contains(key("a", 2)));
  EXPECT_EQ(900u, cache.bytes());
  EXPECT_LE(cache.bytes(), 1000u);
}

TEST(SkidCommsScanCache, ReinsertingAKeyReplacesRatherThanDoubleCounts) {
  ScanCache cache(testConfig());
  cache.insert(key("a", 0), 100);
  cache.insert(key("a", 0), 250);
  EXPECT_EQ(1u, cache.size());
  EXPECT_EQ(250u, cache.bytes());
}

TEST(SkidCommsScanCache, RejectsAnEntryLargerThanTheWholeBudget) {
  ScanCache cache(testConfig());
  cache.insert(key("a", 0), 400);

  // Emptying the cache would still not make room, so the oversized scan is
  // refused instead and comes back as its own eviction.
  const std::vector<ScanKey> evicted = cache.insert(key("a", 1), 5000);
  ASSERT_EQ(1u, evicted.size());
  EXPECT_EQ(key("a", 1), evicted[0]);
  EXPECT_FALSE(cache.contains(key("a", 1)));
  EXPECT_TRUE(cache.contains(key("a", 0)));
  EXPECT_EQ(1u, cache.rejected());
}

TEST(SkidCommsScanCache, RejectsInvalidKeysAndInvalidConfiguration) {
  ScanCache cache(testConfig());
  const std::vector<ScanKey> evicted = cache.insert(key("", 1), 10);
  ASSERT_EQ(1u, evicted.size());
  EXPECT_EQ(0u, cache.size());

  Config bad = testConfig();
  bad.max_cached_scans = 0;
  ScanCache broken(bad);
  EXPECT_FALSE(broken.error().empty());
  EXPECT_FALSE(broken.insert(key("a", 0), 10).empty());
  EXPECT_EQ(0u, broken.size());
}

TEST(SkidCommsScanCache, EraseAndClearFreeBytes) {
  ScanCache cache(testConfig());
  cache.insert(key("a", 0), 100);
  cache.insert(key("a", 1), 100);

  EXPECT_TRUE(cache.erase(key("a", 0)));
  EXPECT_FALSE(cache.erase(key("a", 0)));
  EXPECT_EQ(100u, cache.bytes());

  cache.clear();
  EXPECT_EQ(0u, cache.size());
  EXPECT_EQ(0u, cache.bytes());
}

// ---------------------------------------------------------------------------
// RequestTracker
// ---------------------------------------------------------------------------

TEST(SkidCommsRequestTracker, ThrottlesBeyondTheInFlightCap) {
  RequestTracker tracker(testConfig());
  ASSERT_TRUE(tracker.error().empty());

  EXPECT_EQ(RequestDecision::kSend, tracker.request(key("a", 0), 0.0));
  EXPECT_EQ(RequestDecision::kSend, tracker.request(key("a", 1), 0.0));
  EXPECT_EQ(2u, tracker.inflight());

  EXPECT_EQ(RequestDecision::kThrottled, tracker.request(key("a", 2), 0.0));
  EXPECT_EQ(1u, tracker.throttled());

  // Completing one frees a slot.
  EXPECT_GE(tracker.complete(key("a", 0), 0.5), 0.0);
  EXPECT_EQ(RequestDecision::kSend, tracker.request(key("a", 2), 0.6));
}

TEST(SkidCommsRequestTracker, ReportsDuplicateRequestsAndRoundTripLatency) {
  RequestTracker tracker(testConfig());
  ASSERT_EQ(RequestDecision::kSend, tracker.request(key("a", 0), 10.0));
  EXPECT_EQ(RequestDecision::kAlreadyPending,
            tracker.request(key("a", 0), 10.1));

  EXPECT_NEAR(0.25, tracker.complete(key("a", 0), 10.25), 1.0e-9);
  // A second completion has nothing outstanding to match.
  EXPECT_LT(tracker.complete(key("a", 0), 10.3), 0.0);
  EXPECT_EQ(0u, tracker.inflight());
}

TEST(SkidCommsRequestTracker, RetriesOnceThenAbandons) {
  RequestTracker tracker(testConfig());  // max_request_attempts = 2
  ASSERT_EQ(RequestDecision::kSend, tracker.request(key("a", 0), 0.0));

  // Before the timeout nothing expires.
  EXPECT_TRUE(tracker.expire(4.0).resend.empty());

  const RequestTracker::Expired first = tracker.expire(6.0);
  ASSERT_EQ(1u, first.resend.size());
  EXPECT_EQ(key("a", 0), first.resend[0]);
  EXPECT_TRUE(first.abandoned.empty());
  EXPECT_EQ(1u, tracker.retried());
  EXPECT_TRUE(tracker.pending(key("a", 0)));

  const RequestTracker::Expired second = tracker.expire(12.0);
  EXPECT_TRUE(second.resend.empty());
  ASSERT_EQ(1u, second.abandoned.size());
  EXPECT_EQ(key("a", 0), second.abandoned[0]);
  EXPECT_FALSE(tracker.pending(key("a", 0)));
  EXPECT_TRUE(tracker.isAbandoned(key("a", 0)));
  EXPECT_EQ(1u, tracker.abandoned());

  // An abandoned key is not asked for again until it is explicitly forgotten.
  EXPECT_EQ(RequestDecision::kAbandoned, tracker.request(key("a", 0), 13.0));
  EXPECT_TRUE(tracker.forget(key("a", 0)));
  EXPECT_EQ(RequestDecision::kSend, tracker.request(key("a", 0), 14.0));
}

TEST(SkidCommsRequestTracker, ALateResponseClearsAbandonment) {
  RequestTracker tracker(testConfig());
  tracker.request(key("a", 0), 0.0);
  tracker.expire(6.0);
  tracker.expire(12.0);
  ASSERT_TRUE(tracker.isAbandoned(key("a", 0)));

  EXPECT_LT(tracker.complete(key("a", 0), 13.0), 0.0);
  EXPECT_FALSE(tracker.isAbandoned(key("a", 0)));
}

TEST(SkidCommsRequestTracker, RejectsUnusableKeysAndTimestamps) {
  RequestTracker tracker(testConfig());
  EXPECT_EQ(RequestDecision::kInvalidKey, tracker.request(key("", 0), 0.0));
  EXPECT_EQ(RequestDecision::kInvalidKey,
            tracker.request(key("a", 0), std::nan("")));
  EXPECT_EQ(0u, tracker.inflight());
}

// ---------------------------------------------------------------------------
// DeferredCandidateQueue
// ---------------------------------------------------------------------------

TEST(SkidCommsDeferredCandidates, ReleasesOnlyWhenEveryScanHasArrived) {
  DeferredCandidateQueue queue(testConfig());
  ASSERT_TRUE(queue.error().empty());

  DeferredCandidate candidate;
  candidate.query_bin = 7;
  candidate.candidate_bin = 2;
  candidate.sector_shift = 4;
  candidate.parked_at_s = 1.0;
  candidate.missing = {key("a", 7), key("b", 2)};
  ASSERT_TRUE(queue.park(candidate));

  EXPECT_TRUE(queue.release(key("a", 7)).empty());
  EXPECT_EQ(1u, queue.size());

  const std::vector<DeferredCandidate> ready = queue.release(key("b", 2));
  ASSERT_EQ(1u, ready.size());
  EXPECT_EQ(7, ready[0].query_bin);
  EXPECT_EQ(2, ready[0].candidate_bin);
  EXPECT_EQ(4, ready[0].sector_shift);
  EXPECT_EQ(0u, queue.size());
}

TEST(SkidCommsDeferredCandidates, DropsTheOldestWhenFull) {
  DeferredCandidateQueue queue(testConfig());  // capacity 3
  for (int i = 0; i < 4; ++i) {
    DeferredCandidate candidate;
    candidate.query_bin = i;
    candidate.candidate_bin = 100 + i;
    candidate.parked_at_s = static_cast<double>(i);
    candidate.missing = {key("a", i)};
    ASSERT_TRUE(queue.park(candidate));
  }
  EXPECT_EQ(3u, queue.size());
  EXPECT_EQ(1u, queue.dropped());

  // The dropped candidate can no longer be released.
  EXPECT_TRUE(queue.release(key("a", 0)).empty());
  EXPECT_EQ(1u, queue.release(key("a", 1)).size());
}

TEST(SkidCommsDeferredCandidates, RejectsCandidatesThatCannotBeCompleted) {
  DeferredCandidateQueue queue(testConfig());

  DeferredCandidate nothing_missing;
  nothing_missing.missing = {};
  EXPECT_FALSE(queue.park(nothing_missing));

  DeferredCandidate bad_key;
  bad_key.missing = {key("", 3)};
  EXPECT_FALSE(queue.park(bad_key));

  DeferredCandidate bad_time;
  bad_time.parked_at_s = std::nan("");
  bad_time.missing = {key("a", 3)};
  EXPECT_FALSE(queue.park(bad_time));

  EXPECT_EQ(0u, queue.size());
}

TEST(SkidCommsDeferredCandidates, ExpiresStaleCandidates) {
  Config config = testConfig();
  config.max_deferred_age_s = 10.0;
  DeferredCandidateQueue queue(config);

  DeferredCandidate old_candidate;
  old_candidate.parked_at_s = 0.0;
  old_candidate.missing = {key("a", 0)};
  ASSERT_TRUE(queue.park(old_candidate));

  DeferredCandidate fresh;
  fresh.parked_at_s = 20.0;
  fresh.missing = {key("a", 1)};
  ASSERT_TRUE(queue.park(fresh));

  EXPECT_EQ(1u, queue.expire(25.0));
  EXPECT_EQ(1u, queue.size());
  EXPECT_EQ(1u, queue.expired());
  EXPECT_TRUE(queue.release(key("a", 0)).empty());
  EXPECT_EQ(1u, queue.release(key("a", 1)).size());
}

// ---------------------------------------------------------------------------
// TransferStats
// ---------------------------------------------------------------------------

TEST(SkidCommsTransferStats, AccumulatesBytesMessagesAndLatency) {
  TransferStats stats;
  EXPECT_TRUE(std::isnan(stats.meanLatencySeconds()));
  EXPECT_TRUE(std::isnan(stats.maxLatencySeconds()));

  stats.recordSent(400);
  stats.recordSent(600);
  stats.recordReceived(1024);
  stats.recordLatency(0.10);
  stats.recordLatency(0.30);

  EXPECT_EQ(2u, stats.messagesSent());
  EXPECT_EQ(1000u, stats.bytesSent());
  EXPECT_EQ(1u, stats.messagesReceived());
  EXPECT_EQ(1024u, stats.bytesReceived());
  EXPECT_EQ(2u, stats.latencySamples());
  EXPECT_NEAR(0.20, stats.meanLatencySeconds(), 1.0e-12);
  EXPECT_NEAR(0.30, stats.maxLatencySeconds(), 1.0e-12);
  EXPECT_NE(std::string::npos, stats.summary().find("latency"));
}

TEST(SkidCommsTransferStats, IgnoresUnusableLatencySamples) {
  TransferStats stats;
  stats.recordLatency(-1.0);
  stats.recordLatency(std::nan(""));
  EXPECT_EQ(0u, stats.latencySamples());
  EXPECT_EQ(2u, stats.rejectedLatencySamples());
  EXPECT_TRUE(std::isnan(stats.meanLatencySeconds()));
  EXPECT_NE(std::string::npos, stats.summary().find("no latency samples"));

  stats.recordLatency(0.5);
  EXPECT_EQ(1u, stats.latencySamples());
  EXPECT_NEAR(0.5, stats.maxLatencySeconds(), 1.0e-12);

  stats.reset();
  EXPECT_EQ(0u, stats.messagesSent());
  EXPECT_TRUE(std::isnan(stats.maxLatencySeconds()));
}
