# Distributed Keyframe Graph Change Record

Date: 2 September 2026

Branch: `v3`

Paper target: Equations (6) and (7) in `2505.08230v3`

Commits:

| Commit | Subject |
|---|---|
| `b41a338` | Prepare pose graph for remote keyframe states |
| `d0ac2e7` | Implement direct distributed keyframe factors |

## Why this change exists

The paper's per-robot optimizer does not eliminate the other robots from an
inter-robot loop. For robot `alpha`, Equation (6) optimizes its locally owned
keyframes together with the subset of peer keyframes linked by inter-robot
closures; Equation (7) names that augmented state. The previous package did
something different: map fusion waited for two registrations, differenced the
peer frame away, and published a factor between two local poses. That was a
useful approximation, but it omitted remote states, ignored a lone accepted
registration, and could not transfer corrections through an explicit shared
keyframe graph.

This change replaces that default route with direct cross-robot factors. The
old algebraic route remains available behind
`mapfusion.interRobot.direct_keyframe_factors: false` for controlled
comparisons.

## State and factor semantics

Each mapping process now keeps the following variables in its existing iSAM2
instance:

- Locally owned poses use `X(index)`, represented by GTSAM symbol `x`.
- A peer pose uses `(robot_id, keyframe_index)`, represented internally by a
  process-local packed `r` symbol. Peer names remain explicit on the ROS
  contract; their numeric namespaces are not exchanged.
- Only peer poses that occur in accepted inter-robot closures are inserted.
  This is the partial remote state in Equation (7), not a copy of every peer
  keyframe.

An accepted registration is published as one
`BetweenFactor(from, to, measurement)` with its Hessian-shaped full 6-by-6
covariance. Both endpoint robots receive the same oriented factor. The
recipient's local endpoint uses its `x` key and the other endpoint uses its
peer `r` key.

The endpoint poses carried in `LoopConstraint` are estimates in each owner's
own map frame. Absolute poses from different owner maps are never composed.
For successive observed keyframes from one peer, their owner-frame relative
motion is frame invariant, so it supplies a sparse peer-trajectory edge. The
trajectory store accepts out-of-order keyframes and adds one edge per newly
observed pose after the first, producing a bounded spanning tree over the
observed subset.

A new remote value needs an initial estimate before iSAM2 can accept the
factor. It is seeded from the current local estimate and the registration:

```text
remote_initial = local * z                  when local is `from`
remote_initial = local * inverse(z)         when local is `to`
```

That seed is not a prior. Adding a prior made from the same registration would
count the measurement twice. Consequently, one cross-robot factor introduces
and locates a free peer variable but cannot by itself correct the local
trajectory. A second cross-robot factor connected through a peer-motion edge
forms the first correction cycle, which is the expected gauge behaviour.

## Contract and delivery changes

`LoopConstraint.msg` now names both endpoint robots and carries each
endpoint's owner-frame pose. `robot_id` remains the recipient. Existing
same-robot producers set both endpoint IDs to the recipient through the
original `populate()` helper; new inter-robot producers use
`populateInterRobot()`.

Map fusion publishes every newly PCM-accepted registration directly to the
observer and to the peer. Endpoint-pair identities are canonicalized, so the
same physical registration observed in the opposite direction is not added
twice. The factor publishers, mapping subscriber, and ZeroMQ bridge use a ROS
queue depth of 100 because one PCM update can release a clique-sized burst.

This is source compatibility for callers using the helper, not ROS wire
compatibility. Adding fields changes the message type description, so bags or
separately built nodes using the old `LoopConstraint` schema must be rebuilt or
converted.

## Relationship to map and Earth frames

The distributed graph does not assume that the platforms' map origins are
geographically close. Direct registrations constrain sensor keyframes, and
peer-motion edges are differences inside one peer's own map, so neither
requires subtracting large global coordinates or composing unrelated local
maps.

The existing map-fusion alignment graph remains responsible for publishing
fleet-map-to-platform-map transforms for visualization and coordination. It no
longer mediates keyframe factors when direct mode is enabled. The earlier
`earth -> platform/map -> platform/odom -> platform/base_link` contract and
UTM/UPS zone-qualified outputs therefore remain separate from Equation (6):
Earth/ECEF supplies geographic placement, while direct inter-robot
registrations supply graph constraints.

## Deliberate limits

This is structural Equation (6)/(7) parity, not yet a claim of experimental
paper parity:

1. The sparse peer-motion edges are derived from announced optimized poses
   and use configured rotation/translation uncertainty floors. They are not
   the peer's raw odometry factors with propagated covariance.
2. A peer pose is a snapshot. Later corrections made by the owner are not
   streamed back to revise an already stored peer edge.
3. The recipient imports neither the peer's GPS factors nor its intra-robot
   loop factors; it holds only the observed spanning tree and direct
   cross-robot registrations.
4. iSAM2 is add-only here. If a later PCM maximum clique excludes a factor
   that was accepted earlier, the installed factor is not retracted.
5. Delivery is reliable while DDS/ZeroMQ peers are connected, but the
   publishers are volatile and there is no acknowledgement/replay protocol
   for a peer that was offline during publication.
6. The remote-motion floors (`0.05` rad and `0.20` m by default), PCM gates,
   and registration noise bounds still need dataset calibration.

These limits are intentionally visible rather than hidden behind the old
two-level equivalence argument. The next fidelity step is to exchange
revisioned peer odometry/loop factors or corrected peer states, then measure
the result on real multi-robot field data.

## Verification boundary

Completed on ROS 2 Lyrical on 2 September 2026:

- A full `liorf` configure, compile, interface generation, link, and install
  completed, including `liorf_mapOptmization`, `liorf_mapFusion`, and
  `liorf_zmqBridge`.
- All 13 non-socket CTest targets passed.
- `test_skid_transport` passed all 25 cases with loopback socket access.
- `test_skid_graph_keys` covers explicit local keys, multiple peer namespaces,
  index bounds, out-of-order sparse trajectories, both factor orientations,
  reversed-endpoint deduplication, and a solver-level two-robot graph in which
  direct factors correct the local trajectory.
- `test_loop_constraint_utils` covers endpoint identities, owner poses,
  covariance ordering, and same-robot compatibility.

After the graph implementation was committed, the provenance-preserving
HelmDyn08/09 fixture was generated at
`RESPLE_dataset/HelmDyn/HelmDyn08_09_two_robot_v1`. `ros2 bag info` reports
61,157 messages over 141.768 seconds on four namespaced Livox topics. The
1.1 GiB artifact also contains 13,040 and 11,544 retimed ground-truth poses,
respectively, plus `PROVENANCE.json` with source and output SHA-256 hashes. The
generator opened both source databases read-only, completed through its
partial-directory/atomic-move path, and left no partial directory behind.

Derivation validates the fixture and its timestamp/topic contract, not the
graph runtime. This change has not yet exercised the direct factor route
through two live ROS pipelines, RViz, or a field radio, and no
trajectory-accuracy claim is made.
