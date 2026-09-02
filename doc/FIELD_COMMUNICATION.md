# Field Communication

The paper deploys over ROS plus ZeroMQ (Section VI-B) rather than over a single
shared DDS domain. This document is the deployment that corresponds to.

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

## Verifying a deployment

Do this on a bench before a field trial:

1. **Two bridges on one host.** Run both with different `ROS_DOMAIN_ID`s,
   binding to `tcp://127.0.0.1:7447` and `tcp://127.0.0.1:7448` and pointing
   at each other. `ros2 topic hz /solid/context_info` in each domain should
   show the other's announcements.
2. **Check the bridge's own report.** Every `report_period_s` it logs messages
   and bytes each way, plus what it dropped and why. `dropped own` climbing is
   normal in a full mesh; `topic mismatch` climbing means a peer is publishing
   a topic that only prefix-matches a subscription; `send failures` climbing
   means the send high-water mark is being hit.
3. **Confirm the loop is broken.** `echoes suppressed` should climb roughly in
   step with received messages. If it stays at zero while both robots are
   publishing, the suppression window is too short and traffic is amplifying.
4. **Then separate the hosts** and repeat with the real radio.

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

The `liorf_zmqBridge` node was compiled successfully with the complete package
on ROS 2 Lyrical on 1 September 2026. It has **not** been run as a bridge or
tested across two ROS domains/hosts. Follow the bench verification above before
deploying it.
