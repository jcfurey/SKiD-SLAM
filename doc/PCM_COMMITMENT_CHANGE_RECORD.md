# PCM and Two-Robot Stabilization Change Record

Date: 2 September 2026

Branch: `v3`

Commit: `774594a` (*Stabilize PCM publication and matcher diagnostics*)

Paper target: registration robustness and Equation (11) pairwise consistency
in `2505.08230v3`

## Why this change exists

The first two-pipeline HelmDyn08/09 replay reached the paper registration path,
but it exposed four problems that a single-robot replay could not:

1. Every coarse registration constructed KISS-Matcher with effective ROBIN and
   solver noise bounds above its 1.0 m large-map clamp. KISS-Matcher warned on
   every construction, flooding both map-fusion and mapping logs.
2. Expected descriptor false positives were logged as node-level warnings.
   The rejection reason already existed in `LoopDiagnostic`, so the console
   repeated information without adding an actionable fault.
3. The map-fusion implementation stored each registration as
   `source <- target`, while paper Equation (11) closes its cycle with
   `target <- source`. Identity-like examples concealed the direction error.
4. A current maximum clique was published immediately even though PCM is
   recomputed as new candidates arrive and the receiving iSAM2 graph cannot
   retract a factor. A short-lived or undersized clique could therefore become
   permanent graph state.

The replay also exposed a measurement-provenance defect. Descriptor
announcements leave a FIFO on a background thread, but `publishContextInfo()`
copied the node's latest cloud header when the FIFO was drained. An older
keyframe could consequently be announced with a newer scan's timestamp. The
keyframe index and geometry remained intact, but diagnostics and ground-truth
association could be wrong by tens of seconds.

## Implemented behaviour

### Matcher bounds and diagnostics

KISS-Matcher multiplies each noise-bound gain by the coarse voxel size. With
the default 2.0 m voxel, the gains are now 0.5 for ROBIN and 0.375 for the
solver, giving explicit 1.0 m and 0.75 m bounds. Validation rejects a
clamping-enabled configuration whose effective bound exceeds 1.0 m. A user
who intentionally wants a larger unclamped value must explicitly set
`coarse_clamp_noise_bounds: false`.

Registration failures caused by weak descriptor candidates now use DEBUG
console logging. Every attempt and its correspondence, inlier, overlap,
truncated-MSE, uncertainty, and timing data remain available on the structured
`LoopDiagnostic` topic.

### Correct Equation (11) direction

Before calling the covariance-aware PCM residual, map fusion now inverts both
stored inter-robot measurements and propagates their covariances. A nonidentity
two-robot cycle test uses translated and rotated poses to prove that the
correct direction closes the cycle and that the old direction does not.

### Provisional cliques and committed factors

The current PCM maximum clique is now explicitly provisional. A candidate is
committed only after both conditions hold:

- the complete clique contains at least `pcm_min_publish_clique_size`
  candidates; and
- that candidate remains in a supported clique for
  `pcm_min_publish_observations` consecutive recomputations.

Time spent in an undersized clique does not count toward the observation
requirement. Leaving the current clique resets an uncommitted candidate's
streak. Once committed, a candidate remains committed because the current
factor transport and iSAM2 consumer are add-only. Only this monotonic committed
set may drive map-alignment estimation or either the direct or compatibility
factor publisher.

`LoopDiagnostic.pcm_accepted` reports membership in the current clique;
`pcm_committed` reports permission to affect graph state. Diagnostic reasons
distinguish provisional membership, commitment, ordinary exclusion, and later
exclusion of an already committed candidate.

`mapfusion.interRobot.publish_factors` is an explicit evaluation switch. When
false, descriptor retrieval, scan transfer, registration, PCM, diagnostics,
and map-alignment estimation still run, but neither factor route publishes.
The node emits one startup warning so a dry run cannot be mistaken for a
normal deployment.

### Runtime artifacts and timestamps

The maximum-clique helper still exchanges its graph through a Matrix Market
file. Its default directory is now `${TMPDIR:-/tmp}/skid_slam_pcm`, created by
the launch file, rather than the installed package's `config/` directory. This
prevents a symlink install from modifying a tracked source file during replay.

Outgoing `ContextInfo` now retains `SOLiDBin.time`, the timestamp captured with
the descriptor and keyframe, while preserving the latest header's frame ID.
The corrected run produced 122 endpoint timestamp mappings with no conflicts
or per-robot key-index monotonicity violations.

## Parameters

| Parameter | Package default | HelmDyn08/09 profile | Meaning |
|---|---:|---:|---|
| `mapfusion.interRobot.pcm_min_publish_clique_size` | 5 | 12 | Minimum supported current clique |
| `mapfusion.interRobot.pcm_min_publish_observations` | 3 | 3 | Consecutive supported memberships before commitment |
| `mapfusion.interRobot.pcm_max_translation_residual_m` | 0.0 | 1.0 m | Covariance-independent cycle-translation ceiling; zero disables it |
| `mapfusion.interRobot.pcm_max_rotation_residual_rad` | 0.0 | 0.0 | Covariance-independent cycle-rotation ceiling; zero disables it |
| `mapfusion.interRobot.publish_factors` | `true` | `true` | Allow committed measurements to mutate endpoint graphs |
| `mapfusion.interRobot.diagnostic_qos_depth` | 1000 | 1000 | Audit-message queue depth for a PCM recomputation burst |
| `mapfusion.registration.coarse_robin_noise_bound_gain` | 0.5 | 0.5 | ROBIN bound divided by coarse voxel size |
| `mapfusion.registration.coarse_solver_noise_bound_gain` | 0.375 | 0.375 | Solver bound divided by coarse voxel size |

The HelmDyn profile also requests five SOLiD candidates, starts PCM after ten
registrations, uses a 1.0 Mahalanobis-distance PCM gate, and retains a
1.5 m² truncated-MSE limit. A 1.0 m² trial removed an early partial-overlap
anchor and allowed a coherent alias mode to bootstrap, so delayed commitment
remains protection against transient modes. A later complete-run audit found
that covariance inflation let one 2.9 m cycle inconsistency satisfy the
normalized gate. The independent 1.0 m translation ceiling closes that route
without changing factor covariance. See
[`PCM_ABSOLUTE_GATE_CHANGE_RECORD.md`](PCM_ABSOLUTE_GATE_CHANGE_RECORD.md).

## Verification

The package rebuilt from source on ROS 2 Lyrical. Fourteen non-socket CTest
targets passed, including new commitment-policy and nonidentity PCM-direction
coverage. The ZeroMQ transport target passed all 25 cases with loopback socket
access.

A corrected, factor-enabled 70-second bag-time replay of the synthetic
HelmDyn08/09 fixture ran both estimator/map-fusion pipelines at 2x:

- each Livox bridge received and published 1,400 LiDAR messages and 14,001 IMU
  messages, with no point-count mismatch, middleware loss, or incompatible
  QoS report;
- the structured trace contained 8,566 descriptor candidates and 86 accepted
  registrations;
- PCM produced 23 commitment transitions and 37 unique direct factors; all 37
  appeared on each endpoint robot's `LoopConstraint` topic;
- the INFO console contained no KISS noise-bound warnings and no expected
  registration-rejection warnings;
- of the 37 published factors, 30 had both endpoint timestamps within 30 ms of
  a retimed mocap sample. All 30 had translation RTE below 1.33 m, with
  0.16 m median, 0.40 m p90, and 0.57 m p95 error.

The other seven factors are not included in that strict metric because at
least one endpoint lies in a mocap sampling gap. A nearest-sample check with a
loose 0.5-second association finds 36 of 37 below 2 m, but that is diagnostic
evidence rather than a paper-comparable score. A separate factor-disabled run
gave SE(3)-aligned translation ATE RMSE of 0.056 m for jackal0 and 0.068 m for
jackal1, confirming that the local translation trajectories were not the
source of a large mismatch.

A subsequent complete-timeline run preserved 111 factors on each endpoint
topic. The checked-in factor auditor found exact two-sided delivery and
strictly associated 96 of them with ground truth: 0.145 m median and 0.306 m
p90 RTE, with two values above 2 m. The complete evidence, outlier boundary,
and follow-up runtime fixes are recorded in
[`FULL_TWO_ROBOT_REPLAY_CHANGE_RECORD.md`](FULL_TWO_ROBOT_REPLAY_CHANGE_RECORD.md).

## Remaining boundary

The complete synthetic fixture has now run, but it is still two independently
recorded trajectories made simultaneous in one arena, not real multi-robot
field data, and it does not exercise the radio transport. The HelmDyn
ground-truth orientations contain sign-invariant near-180-degree discontinuities
and are now explicitly treated as position-only, so RRE and ARE are not
claimed. Committed factors still cannot be retracted if a persistent false
mode later displaces them; revisioned peer state/factor exchange and graph
factor removal remain the robust long-term fix.
