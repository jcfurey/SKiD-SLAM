# Full Two-Robot Replay and Factor Audit

Date: 2 September 2026

Branch: `v3`

Implementation commits:

- `44e07df` — *Add distributed factor audit tooling*
- `2a01982` — *Fix map-fusion replay bookkeeping*

Fixture: synthetic `HelmDyn08_09_two_robot_v1`, 61,157 messages over
141.768334854 seconds

## Why this follow-up exists

The earlier 70-second run established that delayed PCM commitment could
publish the same factors to both endpoint graphs, but it left the second half
of the fixture untested and factor scoring depended on one-off analysis. This
follow-up closes the synthetic complete-timeline gap, preserves the evidence
outside `/tmp`, and makes factor delivery and ground-truth scoring repeatable.

It also records two runtime defects that only became visible in a long run:

1. every local keyframe entered both per-peer announcement queues even when a
   peer was absent or this robot was the receiving endpoint; and
2. a newly optimized fleet-map alignment reused the timestamp of the
   keyframe that triggered it, which could be older than TF already broadcast
   for the same edge.

## Preserved evidence

Three isolated ROS domains were used so no other `/clock` or SKiD graph could
enter a run:

| Domain | Purpose | Result |
|---:|---|---|
| 193 | 1x complete-input run | Both bridges consumed every source message: jackal0 2,836 LiDAR/28,355 IMU and jackal1 2,724 LiDAR/27,242 IMU, with no reported loss or mismatch. A recorder invocation error prevented a diagnostic bag, but the run exposed one stale fleet-map TF update. |
| 194 | 1x complete-timeline diagnostic capture | Preserved 59,659 output messages over 141.605 seconds. Jackal1 consumed every input; jackal0 missed its first 18 LiDAR and 175 IMU messages during startup. The captured graph-facing evidence remains valid, but this is not an exact-input-delivery run. |
| 195 | 2x post-fix complete-timeline smoke test | All ten periodic communication reports showed `announcement backlog dropped 0/0`; the log contains no `TF_OLD_DATA`, oversized KISS bound, expected registration-rejection warning, ERROR, or FATAL entry. It published 95 direct factors before EOF and left no replay process behind. |

The domain 194 artifact is under:

```text
build/skid_validation/helmdyn08_09_full_20260902_domain194/
```

It contains the 43.8 MiB MCAP, launch/record/player logs, extracted candidate
and accepted-registration CSVs, and strict and loose JSON factor reports. The
MCAP contains:

- 53,964 structured diagnostics;
- 24,633 eligible descriptor-candidate rows;
- 209 accepted registrations;
- 111 `LoopConstraint` messages on each endpoint topic;
- 2,765 jackal0 and 2,708 jackal1 mapping-odometry messages.

The post-fix domain 195 console log is under:

```text
build/skid_validation/helmdyn08_09_postfix_20260902_domain195/launch.log
```

These runtime artifacts are intentionally ignored by Git.

## Reproducible factor audit

`evaluation/score_loop_factors_from_bag.py` reads both endpoint factor topics
and the observer's accepted-registration diagnostics. It canonicalizes factor
orientation, verifies identical measurement delivery to both endpoints,
rejects duplicates or wrong recipients, audits keyframe-time consistency, and
associates each endpoint's original timestamp with retimed TUM ground truth.
It writes a machine-readable report and can enforce RTE, RRE, and minimum
association gates.

For the domain 194 MCAP, the transport contract is exact:

- 111 unique factors on each endpoint topic;
- no missing, additional, duplicate, or numerically different measurement;
- every factor addressed once to each endpoint robot;
- 282 unique endpoint timestamps, with no conflict or key-index regression.

The ground-truth results are:

| Association | Factors | RTE median | RTE p90 | RTE p95 | RTE maximum |
|---|---:|---:|---:|---:|---:|
| at most 30 ms per endpoint | 96/111 | 0.145 m | 0.306 m | 0.444 m | 2.924 m |
| at most 500 ms per endpoint | 111/111 | 0.151 m | 0.352 m | 0.801 m | 2.924 m |

This is stronger evidence than the earlier bounded run, but not a clean
accuracy result. The strict set contains two RTE values above 2 m. Factor
`jackal0/164 <-> jackal1/59` also has 2.735 m endpoint-separation error and is
a genuine geometry outlier worth investigating. Factor
`jackal0/457 <-> jackal1/258` has 2.208 m RTE but only 0.021 m separation
error; its vector error is therefore sensitive to the unresolved HelmDyn
body/LiDAR orientation convention. Several otherwise position-consistent
factors cross discontinuities in the supplied ground-truth orientation and
produce near-180-degree RRE. RRE is reported for diagnosis but is not claimed
as a calibrated result.

## Runtime corrections

The lower numeric robot ID is the sole announcement producer for each pair.
That ownership rule now gates both enqueue and dequeue, so an unconfigured or
receive-only route never consumes backlog capacity or reports fictitious
drops. The rule is isolated in `skid_comms.hpp` and unit tested for equal,
reversed, multi-digit, and absent IDs.

Fleet-map alignment messages now use the time at which the optimized estimate
is published. Measurement diagnostics retain original keyframe timestamps;
state-update TF messages do not. The same correction is applied to the legacy
Scan Context comparison backend and compatibility odom-alignment output.

`module_loam.launch.py` now exposes node respawn as a launch argument. Normal
launches retain the previous `true` default, while the two-robot bag playback
sets it false so teardown cannot resurrect front-end and IMU processes after
their supervisor has begun shutting down.

## Verification boundary

`liorf` and `skid_slam_playback` rebuilt from source on ROS 2 Lyrical. All 14
non-socket CTest targets pass, including 23 communication-policy cases and the
factor-audit and launch-parameter suites. The source-level evaluation and
launch tests contain 71 passing Python cases. The ZeroMQ socket suite was not
rerun because this slice does not change the transport.

This remains a synthetic pairing of independently recorded trajectories, not
a simultaneous multi-robot or radio result. The next accuracy work is to
resolve the ground-truth body/LiDAR convention, investigate the persistent
`164 <-> 59` geometry outlier, and support revision or retraction when a later
PCM clique invalidates an already committed factor. Field evaluation and
ZeroMQ radio measurements remain open.
