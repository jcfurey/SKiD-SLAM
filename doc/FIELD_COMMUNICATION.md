# Field Communication

The paper deploys over ROS plus ZeroMQ (Section VI-B) rather than over a single
shared DDS domain. This document describes the corresponding deployment.

## Why not just share a DDS domain

Running every robot in one ROS 2 domain works on a bench and tends to fail in
the field:

- **Discovery.** Default DDS discovery is multicast. Field radios and mesh
  links frequently do not forward multicast, and where they do, discovery
  traffic scales badly with participant count.
- **Subnets.** Robots on separate subnets, or behind NAT, are not discoverable
  by default at all.
- **Churn.** A robot that drops off and rejoins forces rediscovery across the
  fleet.
- **Back pressure.** A reliable DDS writer with an unreachable reader retries.
  On a shared link that costs bandwidth precisely when there is none.

A ZeroMQ PUB/SUB mesh replaces discovery with explicit endpoints, crosses
subnets with plain TCP, tolerates churn because a disconnected peer is simply a
socket that reconnects, and drops rather than retries when a peer is
unreachable.

## Architecture

```
robot A                                    robot B
  mapFusion  --ROS--\                    /--ROS--  mapFusion
                     zmqBridge <==TCP==> zmqBridge
  mapOptmization ---/                    \--- mapOptmization
```

Each robot runs one `liorf_zmqBridge`. It binds one ZeroMQ PUB socket, connects
a SUB socket to each peer's PUB endpoint, and moves the inter-robot topics
between the local ROS graph and the mesh. Local topics -- odometry, the local
map, TF -- never cross it.

Carried by default, all derived from `mapfusion.interRobot.solid_topic`:

| Topic | Type | Carries |
|---|---|---|
| `<solid>/context_info` | `liorf/msg/ContextInfo` | SOLiD descriptor announcements |
| `<solid>/scan_request` | `liorf/msg/ScanRequest` | On-demand scan requests |
| `<solid>/scan_data` | `liorf/msg/ScanData` | The scans themselves |
| `<solid>/loop_info_global` | `liorf/msg/LoopConstraint` | Accepted inter-robot loop factors |
| `<solid>/trans_odom` | `nav_msgs/msg/Odometry` | Inter-robot map alignment |

The bridge is type-agnostic: it moves serialised bytes through generic
publishers and subscriptions, so carrying a new type is a parameter change, not
a code change.

## Running it

Each robot needs its own endpoints. A three-robot full mesh:

```bash
# on jackal0 (192.168.1.11)
ros2 launch liorf run_zmq_bridge.launch.py robot:=jackal0 \
    bind_endpoint:=tcp://0.0.0.0:7447 \
    peers:=tcp://192.168.1.12:7447,tcp://192.168.1.13:7447

# on jackal1 (192.168.1.12)
ros2 launch liorf run_zmq_bridge.launch.py robot:=jackal1 \
    bind_endpoint:=tcp://0.0.0.0:7447 \
    peers:=tcp://192.168.1.11:7447,tcp://192.168.1.13:7447

# on jackal2 (192.168.1.13)
ros2 launch liorf run_zmq_bridge.launch.py robot:=jackal2 \
    bind_endpoint:=tcp://0.0.0.0:7447 \
    peers:=tcp://192.168.1.11:7447,tcp://192.168.1.12:7447
```

Set `ROS_DOMAIN_ID` differently on each robot so the local graphs stay
separate and the bridge is the only path between them. Open TCP 7447 between
the robots; nothing else needs to be reachable.

`config/zmq_bridge.yaml` holds the rest, and is the file to edit for
high-water marks, payload bounds and poll rate.

The launch file respawns a failed bridge by default. Use `respawn:=false` for
bounded tests or supervised deployments whose process manager owns restart
policy.

## Verifying a deployment

Do this on a bench before a field trial. The installed integration driver
requires two explicit, distinct domain IDs and two explicit, distinct ports so
a validation run cannot silently collapse into one DDS graph:

```bash
# Replace all four values with domains and ports that are not in use.
ros2 run liorf bench_zmq_bridge.py \
    --domain-a 211 --domain-b 212 \
    --port-a 17453 --port-b 17454
```

The driver starts one bridge in each ROS domain, verifies that both instantiate
the complete default topic/type set, exchanges a real `liorf/msg/ScanRequest`
in each direction, checks the exact payload at the opposite ROS graph, and
requires clean bridge counters with one echo suppressed per direction. It
writes bridge, publisher, and subscriber logs to a new directory under `/tmp`
unless `--artifact-dir` names a new directory explicitly.

Then check the deployment-specific behavior:

1. **Check the bridge's own report.** Every `report_period_s` it logs messages
   and bytes each way, plus what it dropped and why. `dropped own` climbing is
   normal in a full mesh; `topic mismatch` climbing means a peer is publishing
   a topic that only prefix-matches a subscription; `send failures` climbing
   means the send high-water mark is being hit.
2. **Confirm the loop is broken.** `echoes suppressed` should climb roughly in
   step with received messages. If it stays at zero while both robots are
   publishing, the suppression window is too short and traffic is amplifying.
3. **Exercise real traffic.** Run the SLAM publishers long enough to include
   descriptor announcements, requested scans, and a committed-factor burst;
   the one-message driver is a wiring check, not a throughput test.
4. **Then separate the hosts** and repeat over the target radio, including a
   disconnect/reconnect interval.

## Design notes

**PUB/SUB, not REQ/REP or ROUTER/DEALER.** Fan-out is native, an unreachable
peer applies no back pressure to the others, and there is no request/response
state to recover after a link drops. The on-demand scan channel is
request/response at the *application* level, which is where retry and timeout
policy already lives (`mapfusion.comms.*`), so the transport does not need to
duplicate it.

**Explicit endpoints, no discovery.** A robot list is configuration. That is
less convenient than discovery and far more predictable, which is the right
trade for a field deployment.

**Exact topic matching.** ZeroMQ SUB filters subscriptions as prefixes, so a
subscription to `/solid/context_info` would also deliver
`/solid/context_info_extra`. The transport compares the topic exactly on
receipt and counts what it rejects.

**Echo suppression.** A topic bridged in both directions loops: the bridge
publishes a peer's message locally, its own subscription sees it, and it goes
back out. The bridge remembers what it injected, briefly, and drops the echo.
Two genuinely distinct messages with byte-identical payloads on one topic
inside the window would be indistinguishable; serialised ROS messages carry a
header stamp, so this does not arise for the topics carried here.

## Status

The transport (`include/skid_transport.hpp`, `src/skid_transport.cpp`) is
tested over real sockets, including the round trip between two peers, exact
topic matching, self-traffic suppression, empty and multi-megabyte payloads,
publishing with no peer attached, and the echo suppressor's loop-breaking
behaviour.

The `liorf_zmqBridge` node builds with the complete package on ROS 2 Lyrical.
On 2 September 2026 it passed bidirectional one-host tests across isolated ROS
domains twice: a generic `std_msgs/msg/String` smoke test on domains 207/208,
then the checked-in production-contract driver on domains 209/210. The latter
instantiated all five default topic types and transferred one
`liorf/msg/ScanRequest` each way. Each bridge finished at one message sent, one
received, no transport drops or failures, and one local echo suppressed.

This closes the one-host/two-domain wiring check. A two-host test, target-radio
throughput/latency measurement, link-loss recovery run, and sustained real SLAM
traffic remain unverified; do not infer those properties from the loopback
result.
