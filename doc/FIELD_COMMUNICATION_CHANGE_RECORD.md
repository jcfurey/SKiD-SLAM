# Change Record: ZeroMQ Field Communication

Date: 1 September 2026; validation continuation 2 September 2026

Branch: `v3` (continued from `claude/v3-paper-parity-wha23w`)

Baseline: commit `90aa278`, *Record the evaluation harness commit hash*

Scope: the `v3` paper-parity item recorded in
[`PAPER_V3_GAP_AUDIT.md`](PAPER_V3_GAP_AUDIT.md) as *P2 — ROS + ZeroMQ field
communication setup (Section VI-B)*, the last untouched row in the audit.

______________________________________________________________________

## 1. Problem

The paper deploys over ROS plus ZeroMQ. The package had neither a transport nor
a reproducible network setup: robots were assumed to share one DDS domain,
which is the arrangement that tends to fail in the field. Default DDS discovery
is multicast, which field radios often do not forward; robots on separate
subnets are not discoverable at all; and a reliable DDS writer with an
unreachable reader retries, spending bandwidth exactly when there is none.

## 2. What changed

### 2.1 The transport

`include/skid_transport.hpp`, `src/skid_transport.cpp`. ROS-free, so it is
testable on its own.

- `PeerTransport` binds one ZeroMQ PUB socket and connects a SUB socket to each
  peer. Messages are three frames: topic, sender id, payload.
- Bounded by configuration: send and receive high-water marks, and a maximum
  accepted payload. PUB drops rather than blocks, so an unreachable robot
  cannot stall the robot trying to reach it.
- `Statistics` counts what was sent, received, and dropped, separated by
  reason: own traffic, topic mismatch, oversize, malformed, send failure.
- `EchoSuppressor` recognises a message the bridge itself injected locally.

### 2.2 The bridge

`src/liorf-DiSO/zmqBridge.cpp` carries the five inter-robot topics between the
local ROS graph and the mesh. It is type-agnostic: generic publishers and
subscriptions move serialised bytes, so carrying a new message type is a
parameter change rather than a code change, and the bridge cannot fall out of
step with a message definition.

### 2.3 Configuration, launch, documentation

`config/zmq_bridge.yaml`, `launch/run_zmq_bridge.launch.py`, and
`doc/FIELD_COMMUNICATION.md` — the topology, a three-robot worked example, the
bench verification to do before a field trial, and the design notes.

The continuation adds `scripts/bench_zmq_bridge.py`, installed as a package
executable. It requires the caller to choose two distinct ROS domains and two
distinct TCP ports, starts and tears down both sides, verifies all five default
generic publisher/subscription types, exchanges a real `ScanRequest` both
ways, and rejects any run whose payload or final transport counters disagree.
`run_zmq_bridge.launch.py` also exposes `respawn` so bounded tests can disable
automatic restart without changing the field default.

## 3. Design decisions

### 3.1 PUB/SUB rather than REQ/REP or ROUTER/DEALER

Fan-out is native, an unreachable peer applies no back pressure to the others,
and there is no request/response state to recover after a link drops.

The scan channel *is* request/response, but at the application level, where its
retry, timeout and abandonment policy already lives (`mapfusion.comms.*`, added
earlier in this branch). Duplicating that in the transport would give two
independent retry policies on one exchange, which is worse than one.

### 3.2 Exact topic matching on receipt

ZeroMQ SUB filters subscriptions as prefixes, so a subscription to
`/solid/context_info` also delivers `/solid/context_info_extra`. That is a
silent mis-delivery: the payload would deserialize as the wrong type or fail
opaquely. The transport compares the topic exactly after receipt and counts
what it rejects. A test publishes a prefix-colliding topic and asserts nothing
is delivered.

### 3.3 Echo suppression by remembered payload

A topic bridged in both directions loops: the bridge publishes a peer's message
locally, its own subscription to that topic sees it, and it goes back out with
this robot as the sender. The peer's bridge does the same, and traffic
amplifies without bound.

The sender-id check alone does not stop this, because the bridge re-stamps the
message as its own on the way out — a subtlety worth stating, since the check
looks sufficient.

Options were separate ingress and egress topic names, which changes the node's
topic contract, or remembering what was injected. The second was taken: it is
local to the bridge and needs no change anywhere else. Its limitation is real
and documented: two genuinely distinct messages with byte-identical payloads on
one topic inside the window are indistinguishable. Serialised ROS messages
carry a header stamp, so this does not arise for the topics carried here.

### 3.4 Optional at build time

`find_path`/`find_library` locate ZeroMQ; without it the bridge and its test
are skipped with a message naming the packages to install, and everything else
still builds. The audit asked for an *optional* adapter, and a hard dependency
would make every non-multi-robot user install ZeroMQ.

The packages are still declared in `package.xml`, so rosdep installs them and
the bridge builds by default. The CMake guard is for environments where rosdep
was not run.

### 3.5 Explicit endpoints, no discovery

A robot list is configuration rather than something negotiated at runtime.
Less convenient than discovery, and far more predictable, which is the right
trade in the field.

## 4. New parameters

All under `zmq`, in `config/zmq_bridge.yaml`. Unlike the rest of this
package's configuration, **this file differs per robot**: `bind_endpoint` and
`peer_endpoints` are platform-specific.

| Parameter | Default | Meaning |
|---|---|---|
| `bind_endpoint` | `tcp://0.0.0.0:7447` | This robot's publisher |
| `peer_endpoints` | `[]` | Every other robot's bind endpoint |
| `send_high_water_mark` | `100` | Outbound messages before ZeroMQ discards |
| `receive_high_water_mark` | `100` | Inbound messages before ZeroMQ discards |
| `max_payload_mib` | `64` | Largest accepted frame |
| `linger_ms` | `0` | Discard unsent messages on shutdown |
| `poll_rate_hz` | `100.0` | How often the subscriber is drained |
| `report_period_s` | `30.0` | Diagnostics interval |
| `qos_depth` | `100` | Local ROS publisher and subscription depth |
| `echo_suppressor_capacity` | `256` | Remembered injections |
| `echo_suppressor_window_s` | `5.0` | How long one is remembered |
| `topics`, `topic_types` | `[]` | Override the carried set; same length |

## 5. Verification

### What was built and run

| Suite | Result | Covers |
|---|---|---|
| `test_skid_transport` | pass (25 cases) | Configuration validation including malformed endpoints and non-positive bounds; a misconfigured transport being inert rather than half-working; binding and reporting a concrete port; a payload carried between two peers over real TCP; delivery of only subscribed topics; rejection of a prefix-colliding topic; suppression of a robot's own traffic in a full mesh; empty and 4 MiB payloads; refusal of an oversize payload; publishing with no peer neither blocking nor failing; prompt return when nothing arrives; batch draining and the batch limit; and seven echo-suppressor cases including the two-way loop it exists to break |
| Complete ROS 2 Lyrical build | pass | `liorf_zmqBridge` and the installed bench driver |
| Complete `liorf` CTest run | pass (15 targets) | All 12 C++ suites, parameter/launch contracts, evaluation, and the socket transport |
| Cross-domain production-contract bench | pass | ROS domains 209 and 210; TCP 17451 and 17452; all five default topic types constructed; one `liorf/msg/ScanRequest` delivered exactly in each direction; final counters on each bridge were sent 1, received 1, all failure/drop classes 0, echoes suppressed 1 |

The socket tests use real TCP on `127.0.0.1` with an OS-assigned port. PUB/SUB
discards anything published before the subscriber finishes connecting, so a
test that publishes once and reads once would be inherently flaky; the helper
retries until the connection is up, bounded so a genuine failure still fails.
**The suite was run 15 times consecutively with no failures** before being
committed, because a flaky socket test is worse than none.

`skid_transport.cpp` compiles clean under `-Wall -Wextra -Wpedantic`.

The definitive run is under the parent workspace at
`build/skid_validation/zmq_bridge_bench_production_20260902_domains209_210`.
Both subscriber stderr files are empty. The earlier generic-message run is in
`build/skid_validation/zmq_bridge_bench_driver_20260902_domains207_208`.

### What was not verified

- No two physical hosts have been bridged.
- No throughput, latency, high-water-mark, or disconnect/reconnect measurement
  has been made over the target field radio.
- The bounded bench transfers one small production message each way. It
  constructs every default generic type but does not drive sustained SOLiD
  announcements, scan payloads, odometry, or clique-sized factor bursts.

## 6. Follow-up work

The former structural Equation (6) blocker is closed: remote keyframe
variables and direct distributed factors were added later on `v3`. Its
remaining work is fidelity—revisioned peer corrections or original peer
factors/covariances, committed-factor retraction, and real multi-robot
validation—not part of the bridge transport itself.

Communications follow-up:

- Repeat the checked-in bench on two hosts over the target radio, including a
  controlled disconnect/reconnect interval.
- Measure achieved throughput and latency and calibrate the
  high-water marks against it.
- Consider carrying the bridge's own statistics into the evaluation harness's
  communication section, which currently reads only the map-fusion node's
  diagnostics.

## 7. Commits

| Commit | Contents |
|---|---|
| `b71198b` | Add the ZeroMQ peer transport and inter-robot bridge |
| `b526be5` | Add the repeatable cross-domain bench and bridge lifecycle hardening |
| `8f7f0aa` | Exercise the production bridge topic contract |
