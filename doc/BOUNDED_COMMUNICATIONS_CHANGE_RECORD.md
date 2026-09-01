# Change Record: Bounded, On-Demand Inter-Robot Communications

Date: 1 September 2026

Branch: `claude/v3-paper-parity-wha23w`

Baseline: commit `b3aa857`, *Restore retained sources and add the session
change record*

Scope: the `v3` paper-parity item recorded in
[`PAPER_V3_GAP_AUDIT.md`](PAPER_V3_GAP_AUDIT.md) as *P1 — Lightweight message
pool*, and Phase 4 items 1 and 2.

______________________________________________________________________

## 1. Problem

The paper's case for a lightweight descriptor is a communication case: SOLiD is
worth using because it is small enough to exchange continuously between robots
over a constrained field link. The implementation did not realize that.

| Behaviour | Consequence |
|---|---|
| Every `ContextInfo` announcement embedded the complete feature cloud | Steady-state inter-robot traffic scaled with the point cloud, not the descriptor — roughly three orders of magnitude more than the descriptor alone |
| `_context_list_to_publish_1/2` were unbounded `std::vector<SOLiDBin>` | A peer whose link was down accumulated announcements without limit, each pinning a full cloud |
| The announcement backlog was drained newest-first (`back()`/`pop_back()`) | After any backlog built up, older entries could starve permanently |
| `publishContextInfoThread` looped with no sleep and `continue` on empty | The thread spun a core at 100% whenever a peer was idle |
| Received clouds were retained in `_bin_with_id` forever | Memory grew without bound for the length of a mission |
| No accounting | Nothing measured what the link cost, so no claim about it could be checked |

There was also a latent correctness bug: `SOLiD::ptcloud2bin` swaps the cloud
it is handed into the bin it returns, so `bin.cloud` aliased the node's reused
`_laser_cloud_sum` buffer. Every bin queued for announcement therefore pointed
at a buffer that the *next* scan overwrote in place. Queued announcements did
not carry the scan they were built from. This was invisible while nothing
retained a bin for long; it surfaced immediately once scans had to be kept for
later service.

## 2. What changed

### 2.1 New component: `liorf::comms`

`include/skid_comms.hpp`, `src/skid_comms.cpp` hold the communication policy —
what to keep, what to drop, what to ask for, and what to report. Payloads stay
in the node that owns them, so the unit needs no ROS or PCL dependency and the
policy is testable on its own.

| Type | Responsibility |
|---|---|
| `BoundedQueue<T>` | FIFO with a hard capacity; drops and counts the oldest when full |
| `ScanCache` | LRU retention by both entry count and total bytes; returns the keys the caller must release |
| `RequestTracker` | In-flight cap, timeout, bounded retry budget, explicit abandoned state, round-trip latency |
| `DeferredCandidateQueue` | Loop candidates parked until the scans they need arrive; bounded and age-limited |
| `TransferStats` | Message counts, bytes, and latency mean/max per channel |
| `ScanKey` | `(robot_id, keyframe_index)` identity for one platform's keyframe scan |

`ScanCache` deliberately stores sizes and use order only, never point clouds.
That is what lets the eviction rule be tested without a PCL type while leaving
the node as the single owner of the data.

### 2.2 New messages

- `msg/ScanRequest.msg` — requester, owner, keyframe index.
- `msg/ScanData.msg` — owner, requester, keyframe index, an `available` flag,
  and the cloud. `available` is false when the owner's own cache has evicted
  the scan, so the requester stops asking instead of spending its retry budget
  on something that will never arrive.
- `msg/ContextInfo.msg` gains an explicit `int64 keyframe_index`. `scan_cloud`
  is now normally empty and documented as such.

### 2.3 Map-fusion node

`src/liorf-DiSO/mapFusion_so.cpp`:

- Announcements carry the descriptor only, unless
  `mapfusion.comms.announce_scans` restores the old behaviour.
- `ensureCandidateScans()` checks that both scans behind a candidate are held.
  Missing ones are requested; the candidate is parked in the bounded deferred
  queue and resumed by `scanDataHandler()` when the scan lands.
- `run()` is split so `optimizeAndPublish()` (PCM, graph, odometry output) can
  be reached either by an arriving announcement or by a scan that completes a
  parked candidate later.
- One `_scans` store holds every scan the node has, its own and its peers',
  under one `ScanCache` budget. `storeScan()` is the only place scans are added
  or released.
- `buildKDTree()` moves any cloud that arrived with an announcement into that
  store and leaves the bin holding the descriptor and pose only, so a cloud
  never has two lifetimes with only one of them governed by the retention
  policy.
- `publishContextInfoThread` is rate-limited and drains FIFO through
  `publishPendingAnnouncements()`.
- `commsMaintenance()` runs on a timer: resend timed-out requests, abandon
  those past budget, expire stale parked candidates, and log the byte, latency,
  drop, eviction, retry, and abandonment counters.

### 2.4 Build and configuration

- `CMakeLists.txt`: adds the `skid_comms` library, links it into
  `liorf_mapFusion`, registers the two new messages and the new test target.
- `src/liorf-DiSO/config/mapfusion_solid.yaml`: documents the
  `mapfusion.comms.*` block.

## 3. Design decisions

### 3.1 The loopback announcement is gone, not made cheaper

The node used to publish a self-addressed `ContextInfo` and consume it through
its own `solidInfoHandler`, which is how its own places entered the KD-tree.

That does not survive the split. `_pub_context_info` publishes to a topic every
robot subscribes to; `robot_id_receive` is an application-level filter, not a
transport one. A self-addressed announcement carrying a cloud is therefore paid
for on the link exactly like a broadcast, and every peer receives and discards
it. Attaching the scan to the loopback would have left the headline cost
untouched.

Own places now enter the local index directly, and the loopback publish is
removed rather than kept as a descriptor-sized message nothing consumes. The
direct call is gated on `_performs_fusion`, which mirrors the existing
condition guarding whether the announcement subscription is created at all, so
the initial robot's announce-only role is unchanged.

### 3.2 Own and received scans share one budget

An alternative was a separate, larger budget for the node's own scans, on the
grounds that only it can serve them to peers.

One budget was chosen because it makes the memory ceiling a single configured
number rather than a sum the operator has to compute. The cost is real and
worth stating: when a robot's own scan is evicted it can no longer answer a
peer's request for that place, and `ScanData.available` is how it says so.

### 3.3 FIFO instead of newest-first

The old drain took from the newest end. With a bounded queue that starves the
tail permanently, and dropping is already handled at the oldest end by the
queue itself. FIFO plus drop-oldest keeps one consistent rule: under pressure,
the oldest place is lost, and nothing is lost twice.

### 3.4 Abandonment is a state, not a silent give-up

A key whose retry budget is spent stays abandoned until `forget()` is called,
so a silent peer cannot be asked forever. Abandonment is counted and logged.
A later response clears it, because a reply is proof the peer is answering.

That last part was a bug caught by its own test: `complete()` originally
returned early for a key that was no longer pending, so a late reply never
cleared the abandonment it should have.

### 3.5 Byte counts are payload estimates, and say so

`announcementBytes()` and `cloudBytes()` count point data, descriptor, and
identifiers. They do not count serialization or transport framing, so they
under-report the wire cost by a constant per message. They are correct for the
comparison that matters — descriptor-only versus scan-carrying announcements —
and the header says what they are rather than implying a measured wire figure.

## 4. New parameters

All under `mapfusion.comms`, all optional.

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `announce_scans` | bool | `false` | Attach the feature cloud to every announcement (the old behaviour) |
| `announce_rate_hz` | double | `10.0` | Announcement drain rate; one per peer per tick, so it must exceed the keyframe rate |
| `maintenance_period_s` | double | `1.0` | Retry/expiry timer period |
| `report_period_s` | double | `30.0` | Diagnostics logging period |
| `max_pending_announcements` | int | `100` | Per-peer announcement backlog |
| `max_cached_scans` | int | `500` | Retained scans, own and received |
| `max_cached_scan_mib` | int | `512` | Retained scan bytes |
| `max_inflight_requests` | int | `8` | Outstanding scan requests |
| `request_timeout_s` | double | `5.0` | Before a request is retried |
| `max_request_attempts` | int | `3` | Attempts before abandoning a key |
| `max_deferred_candidates` | int | `64` | Candidates parked awaiting scans |
| `max_deferred_age_s` | double | `60.0` | Before a parked candidate is dropped |

## 5. Behavioural changes for operators

- **Steady-state inter-robot traffic drops to descriptor size.** A scan crosses
  the link only behind a descriptor match. Set `announce_scans: true` to
  restore the previous behaviour for comparison.
- **Loop closure now depends on the scan channel.** If `scan_request` or
  `scan_data` does not reach a peer, inter-robot loops stop forming even though
  descriptors still flow. The abandonment warnings and the periodic diagnostics
  line are the place to look.
- **Very old revisits can no longer be registered.** Once a scan is evicted the
  place is still in the descriptor index and still matches, but the
  registration cannot run. Raise `max_cached_scans` and `max_cached_scan_mib`
  for missions whose revisit horizon exceeds the cache.
- **A core is no longer spun.** The announcement thread previously ran a busy
  loop; on a constrained platform this alone may change thermal and power
  behaviour.
- **A new diagnostics line** appears every `report_period_s` with both
  channels' byte and latency figures and every drop, eviction, retry, and
  abandonment counter.

## 6. Verification

### What was built and run

As with the previous slice, no ROS 2 installation was available. The
ROS-independent code was built and tested against Eigen, PCL, and the pinned
solver submodules.

| Suite | Result | Covers |
|---|---|---|
| `test_skid_comms` | pass (21 cases) | Configuration and key validation; bounded-queue drop-oldest and zero capacity; cache LRU eviction, minimum-eviction under the byte budget, re-insert replacement, oversized rejection, invalid keys and configuration, erase/clear; request throttling, duplicate detection, latency, retry-then-abandon, late-response recovery, invalid inputs; deferred release only when complete, drop-oldest, rejection of uncompletable candidates, age expiry; transfer accounting and rejection of unusable latency samples |
| `test_skid_registration` | pass | Unchanged; confirms no regression |
| `test_skid_loop_detection` | pass | Unchanged |
| `test_skid_solid_descriptor` | pass | Unchanged |
| `test_skid_registration_params` | pass | Unchanged |
| `test_observable_scan_match` | pass | Unchanged |

`skid_comms.cpp` and its test compile clean under `-Wall -Wextra -Wpedantic`.

One real bug was found by these tests rather than by inspection: the
late-response abandonment case in §3.4. One test expectation was also wrong —
it assumed the cache would evict every entry that did not fit rather than the
fewest needed — and the corrected test now pins the minimum-eviction property
explicitly.

### What was not verified

**The map-fusion node was not compiled or run.** It requires GTSAM and
`rclcpp`. The diff was reviewed line by line and the policy it drives is
tested, but that is review, not a build.

Specifically still unverified:

1. A ROS 2 Lyrical `colcon build`, including the two new message types.
2. A multi-robot bag replay confirming the request/response round trip, that
   parked candidates resume and produce loop factors, and that the
   `available: false` path is exercised when a scan has been evicted.
3. The actual bandwidth reduction, measured rather than argued.
4. Latency under a realistically constrained link, which is what the
   `request_timeout_s` and `max_request_attempts` defaults should be tuned
   against.
5. That the aliasing fix in §1 behaves as intended over a long run — it is
   correct by construction, but nothing has replayed it.

## 7. Follow-up work

Still open in the audit, unchanged by this session:

- **P1** — Equation (6) keyframe-state semantics in the map-level graph.
- **P2** — Paper dataset configurations.
- **P2** — ZeroMQ field-communication transport or documentation. The message
  contract is now transport-agnostic, which makes a bridge straightforward.
- **P2** — The paper evaluation harness, which is where the bandwidth and
  latency numbers above would become measurements.

Newly opened:

- Calibrate the cache budget against each dataset's revisit horizon.
- Measure achieved bandwidth and latency and compare against `announce_scans:
  true` as the baseline.
- Consider serving a request from a peer that also holds the scan, rather than
  only from its owner, once more than two robots are exercised.

## 8. Commits

| Commit | Contents |
|---|---|
| `040b43a` | Bound inter-robot communications and move scans on demand |
