# SKiD-SLAM Paper v3 Parity Audit

Status: living audit and implementation record for the `v3` branch

Last updated: 1 September 2026 (intra-robot pipeline reuse; bounded on-demand communications; map-alignment uncertainty; paper dataset configurations)

Audit baseline: commit `475b59f`, before the paper-registration work below.
The gap table records that baseline so the provenance problem remains visible;
the implementation-status section records what has since been restored.

Paper: `2505.08230v3`, *SKiD-SLAM: Robust, Lightweight, and
Distributed Multi-Robot LiDAR SLAM in Resource-Constrained Field
Environments* (30 July 2025)

## Change records

Per-change records, each with the decisions behind it and its verification
boundary:

- [`UNIFIED_LOOP_CLOSURE_CHANGE_RECORD.md`](UNIFIED_LOOP_CLOSURE_CHANGE_RECORD.md)
- [`BOUNDED_COMMUNICATIONS_CHANGE_RECORD.md`](BOUNDED_COMMUNICATIONS_CHANGE_RECORD.md)
- [`MAP_ALIGNMENT_UNCERTAINTY_CHANGE_RECORD.md`](MAP_ALIGNMENT_UNCERTAINTY_CHANGE_RECORD.md)

## Executive summary

The code on `dev`/`v3` is not the complete implementation described by the
paper. It is primarily a ROS 2 port of the November 2024 Liorf + Distributed
SOLiD branch, combined with the later SKiD-SLAM documentation and with local
extensions for observable scan matching and geographic frames.

A closer paper-era prototype is still recoverable from Git commit `0611f71`
(`init skid_slam`, 31 March 2025). That commit added KISS-Matcher,
Small-GICP, truncated-fitness code, and field-dataset configurations. Commit
`340ad69`, created about twenty minutes later, removed essentially the entire
implementation. The current ROS 2 reconciliation restored the separate
November 2024 Liorf branch rather than the deleted paper-era tree.

The historical implementation is useful reference material, but it must not
be cherry-picked wholesale. It is ROS 1 code, predates the current frame
contract, and contains correctness issues that need tests and reimplementation.

## What is already implemented

- LiDAR-IMU odometry and local mapping derived from Liorf/LIO-SAM.
- SOLiD descriptor construction, KD-tree inter-robot candidate search, and
  descriptor-based intra-robot candidate search.
- PCM consistency matrices and maximum-clique filtering.
- Per-robot GTSAM pose graphs and a map-level inter-robot alignment graph.
- ROS 2 Jazzy/Lyrical build and launch support.
- RESPLE/X-ICP-inspired observable-subspace local scan matching.
- REP-105 local frame separation and optional ECEF geographic anchoring.
- Descriptor-only announcements with on-demand scan transfer, bounded queues
  and caches, and byte/latency reporting.

The last two items are post-paper extensions and should be retained while
paper parity is restored.

## `v3` implementation status

The registration and measurement-uncertainty slices are implemented on top of
the audit baseline:

- KISS-Matcher coarse global registration now feeds Small-GICP fine
  registration using one explicit `target <- source` transform convention.
- Coarse correspondence, coarse solver-inlier, fine convergence/inlier,
  overlap, and truncated-MSE gates reject weak measurements before PCM.
- Equation (10) is implemented in squared metres. The code does not repeat
  the deleted prototype's fourth-power or double-normalization errors.
- SOLiD map fusion now uses the coarse-to-fine component instead of PCL GICP.
- Accepted-loop publication and mapping subscription both derive from
  `mapfusion.interRobot.loop_topic`; the default is
  `<robot>/context/loop_info`.
- Configuration exposes every registration scale, tolerance, count, and gate
  used by the implementation, with validation during node construction.
- Registration diagnostics report rejection reason, correspondence/inlier
  counts, overlap, truncated MSE, and per-stage timing.
- Small-GICP's final Hessian now supplies the directional shape of a full
  six-degree-of-freedom covariance. Explicit angular and linear standard
  deviations calibrate its physical scale; error, overlap, and weak geometric
  modes can only inflate the configured nominal uncertainty.
- PCM now evaluates Equation (11) as a dimensionless Mahalanobis distance.
  Registration covariance is propagated through the complete SE(3) cycle,
  with separate angular and linear uncertainty for the local trajectory.
- Accepted loops use `LoopConstraint.msg`, with a quaternion pose, full 6x6
  covariance, explicit keyframe indices, and registration diagnostics. The
  mapping back end consumes the covariance as a full GTSAM Gaussian model.
- The message conversion explicitly permutes the ROS covariance convention
  `[tx, ty, tz, rx, ry, rz]` to GTSAM's tangent convention
  `[rx, ry, rz, tx, ty, tz]` and is covered by a round-trip test.
- Intra-robot loops now run the same pipeline. The local mapping node
  describes each keyframe with SOLiD, retrieves revisits from a shared
  descriptor index, and registers every candidate — whether it came from the
  descriptor or from the existing radius search — with the same coarse-to-fine
  registration, the same Equation (10) gate, and the same Hessian-shaped
  covariance. Scan Context and PCL ICP are gone from that node.
- The descriptor distance, the winning circular shift, and the yaw it implies
  are defined once in `skid_loop_detection.hpp` and used by both nodes. The
  registration parameter set is declared once in
  `skid_registration_params.hpp`, so neither node can quietly gain, lose, or
  rename a gate the other still has.
- Intra-robot factors are weighted by the propagated registration covariance,
  optionally behind a Cauchy kernel. That kernel is on by default because
  inter-robot loops are screened by PCM and intra-robot loops are not; it
  bounds a surviving false positive without discarding the covariance shape.

The dependencies are Git submodules pinned to revisions available when the
paper-era implementation was created. They are built from local source; CMake
does not use KISS-Matcher's or ROBIN's configure-time download paths.

| Dependency | Pinned revision | Selection basis |
|---|---|---|
| KISS-Matcher | `14ccb2bfee5450560f07a075d14b2aa61f24be7e` | Latest upstream revision before 1 April 2025 |
| Small-GICP | `2c5e9e6092f06bd8b4a0c618c9714e39e4b315dc` | Latest upstream revision before 1 April 2025 |
| ROBIN | `5bcc76e0aa558bf742ab9ca72ecee113f806a948` | KISS-Matcher's `v.1.2.3` dependency |
| PMC | `66892582006fcd9a225cbe1589461980c04b96a1` | Latest revision preceding the ROBIN pin |
| xenium | `7ee5ed18b858106fc6e0fc19305aebfebce1bdf4` | Exact revision declared by the ROBIN pin |

Verification of the registration and measurement-uncertainty slices on ROS 2
Lyrical includes the mapping and both map-fusion executables, all existing
frame and observable-scan-match tests, exact truncated-MSE tests,
invalid-input tests, covariance/PCM propagation tests, message-order tests,
and a solver-level synthetic test recovering a 2.1-radian rigid rotation
through the complete KISS-Matcher-to-Small-GICP pipeline.

The intra-robot pipeline reuse was verified without a ROS 2 installation:
`skid_registration`, `skid_loop_detection`, `skid_solid_descriptor`, and the
vendored SOLiD translation unit were built and their tests run against Eigen,
PCL, and the pinned KISS-Matcher/Small-GICP/ROBIN/PMC submodules; the ROS
nodes were not compiled or run. The node-level changes still need a ROS 2
Lyrical build and a bag replay before the intra-robot claims above can be
called measured rather than implemented.

The communication path is now bounded and on-demand:

- Announcements carry the SOLiD descriptor only. A scan crosses the link when a
  descriptor match has already justified attempting registration, requested
  through `ScanRequest` and returned through `ScanData`.
- Every buffer that can hold a scan is bounded: the per-peer announcement
  backlog, the scan cache (by both entry count and total bytes), the
  outstanding-request set, and the candidates parked waiting for a scan.
- Requests have an in-flight cap, a timeout, a bounded retry budget, and an
  explicit abandoned state. An owner that no longer holds a scan says so
  instead of letting the requester spend its budget.
- Bytes, message counts, and round-trip latency are reported for both channels,
  alongside drop, eviction, retry, and abandonment counters.

The remaining P1/P2 work is Equation (6) keyframe-state semantics, field
configurations, ZeroMQ deployment documentation, and the paper evaluation
harness.

### Registration covariance model

The paper uses information matrices in Equations (2), (5), (6), and (11), but
does not prescribe how registration covariance is estimated. Small-GICP
exposes a final Hessian `H`; its surface covariances are normalized geometric
models, so treating `H` as an absolutely calibrated information matrix would
be misleading. This implementation uses it only for directional shape:

1. Define `D = diag(sigma_r, sigma_r, sigma_r, sigma_t, sigma_t, sigma_t)`
   from configured nominal angular and linear standard deviations.
2. Eigendecompose the dimensionless shape `D H D`, normalize its eigenvalues
   by the strongest mode, and clamp weak modes by
   `uncertainty_min_information_ratio`.
3. Inflate variance by
   `clamp(max(1, truncated_mse/reference_mse) / overlap, 1, max_scale)`.
4. Transform the normalized inverse shape back with `D` to obtain covariance
   in radians squared, metres squared, and the corresponding cross units.

This is deliberately conservative: the strongest mode cannot become more
confident than the configured nominal uncertainty, while conduit-like weak
geometry, partial overlap, and larger residuals reduce factor weight. The
values require dataset calibration; they are not claimed as paper-provided
sensor statistics.

### Intra-robot loop closure

A file-by-file record of this change, with the decisions behind it and the
exact verification boundary, is in
[`UNIFIED_LOOP_CLOSURE_CHANGE_RECORD.md`](UNIFIED_LOOP_CLOSURE_CHANGE_RECORD.md).

The paper describes one place-recognition and registration pipeline, not one
per loop type. The local mapping node previously ran a different one: Scan
Context retrieval, PCL ICP or GICP alignment, a scalar PCL fitness gate, and
either an isotropic diagonal noise built from that fitness score or a fixed
0.5 m^2 isotropic noise behind a Cauchy kernel. That is now replaced.

What is shared, and where it lives:

| Stage | Shared component | Used by |
|---|---|---|
| Descriptor construction | `liorf::solid::describe` over the vendored SOLiD | both nodes |
| Descriptor distance, circular shift, implied yaw | `liorf::loop_detection::compare` | both nodes |
| Candidate retrieval and revisit exclusion | `liorf::loop_detection::Index` | mapping node |
| Coarse-to-fine registration and Equation (10) | `liorf::registration::registerClouds` | both nodes |
| Registration parameter set and validation | `liorf::registration::declareConfig` | both nodes |
| Covariance propagation through SE(3) | `liorf::uncertainty` | both nodes |

What is deliberately not shared:

- Retrieval structure. Map fusion keeps its libnabo KD-tree over every robot's
  descriptors and its robot-ownership rules; the mapping node uses an exact
  linear scan over its own keyframes. Both rank by Euclidean distance between
  L2-normalized R-SOLiD keys, which is monotonic in the cosine distance the
  shared comparison reports, so they order candidates identically. Retrieval is
  linear in keyframe count, which at loop-closure rates costs far less than the
  registration it feeds.
- Candidate ownership. Map fusion still discards same-robot pairs. That is now
  the correct division of work: the mapping node owns its own revisits and has
  the keyframe clouds and pose graph to close them locally.
- PCM. Inter-robot loops are screened by pairwise consistency maximization
  before reaching a graph. Intra-robot loops are not, so their factor is
  optionally wrapped in a Cauchy kernel (`liorf.loopClosure.robustKernel`,
  on by default). The kernel bounds the influence of a false positive; the
  registration covariance still sets the factor's shape and scale.

Two behavioral notes:

- Both intra-robot clouds are expressed in the map frame using the current,
  still drifted keyframe poses, as the previous radius-search path did.
  KISS-Matcher takes no initial guess, so the drift only has to be small
  enough for the two scans to overlap.
- The coarse yaw guess derived from the A-SOLiD shift was one sector too
  large: the previous expression used `(shift + 1)` sector widths. The shared
  helper uses `shift` sector widths. The value is only a pre-rotation for an
  initial-guess-free global registration, so the correction is small in
  effect, but it is now correct and tested.
- `include/Scancontext.{h,cpp}` and the three vendored headers only that
  detector used (`nanoflann.hpp`, `KDTreeVectorOfVectorsAdaptor.h`,
  `tictoc.h`) are retained in the tree for reference but are no longer
  compiled into any target. The Scan Context baseline itself is unaffected:
  `liorf_mapFusionSC` builds from its own copy under
  `src/liorf-DiSO/third_parties/scanContext`.

The truncated-MSE gate for intra-robot loops defaults to
`historyKeyframeFitnessScore`. Both are mean squared nearest-neighbor
distances in m^2; PCL's is untruncated and therefore never smaller, so a
threshold tuned against it is a permissive bound for the paper's metric rather
than a tighter one. Per-dataset calibration is still outstanding.

### Equation (6) and the two-level formulation

The paper optimizes keyframe states with cross-robot factors. This package
optimizes two levels instead, and this section records exactly where the two
agree and where they do not, so the gap is a known quantity rather than an
assumption.

#### What is actually optimized

**Level 1, per robot.** Each platform's mapping node owns a keyframe graph over
its own poses, with odometry, GPS, intra-robot loop factors, and the
inter-robot-derived factors described below. Keys are the platform's own
keyframe indices; no remote pose is ever a variable.

**Level 2, in map fusion.** For each peer `p`, one SE(3) variable `T_p` -- this
robot's map frame into `p`'s -- is fitted by weighted least squares to the
accepted registrations. For accepted registration `i`, with `S_i` this robot's
keyframe pose in its own frame and `A_i` the same keyframe expressed in `p`'s
frame, the residual is between `T_p^-1 . A_i` and `S_i`, weighted by the
registration covariance.

#### How cross-robot information reaches a keyframe graph

Not as a factor between `X^a_j` and `X^b_k`. Two registrations against the same
peer are differenced to produce a constraint between two of *this* robot's
keyframes:

```
z = (A_that)^-1 . A_this
```

Both `A` terms are expressed in the same peer's frame, so that frame cancels
algebraically. The result is an intra-trajectory factor carrying inter-robot
information, and both registration covariances propagate into it through the
tested SE(3) helpers.

#### Where the formulations agree

For a pair of registrations against **the same peer**, this loses nothing. The
peer's frame cancels exactly rather than approximately, no estimated quantity
mediates, and the factor's covariance is the correct first-order propagation of
the two registration covariances. In the two-robot case that is every
inter-robot factor the system produces.

#### Where they do not

1. **A lone registration contributes nothing.** A single observation of a peer
   has no second observation to difference against, so it never reaches a
   keyframe graph. A joint formulation would use it directly against the remote
   keyframe.
2. **Remote trajectories are not corrected.** Because no remote pose is a
   variable, this robot's observations cannot improve the peer's trajectory,
   and the peer's later corrections cannot revise constraints already emitted.
3. **Shared information is double counted.** Emitted factors that share a
   registration -- and, in the multi-peer case, a map alignment -- are added to
   the graph as independent `BetweenFactor`s. The correlation between them is
   not represented.
4. **Multi-peer factors are mediated by an estimate.** When the two endpoints
   were registered against *different* peers, each must come back through that
   peer's alignment `T_p` before they can be differenced.

Point 4 was previously worse than an approximation: the alignment was composed
in with zero uncertainty, so the resulting factor was overconfident by exactly
the alignment's error. The alignment's marginal is now recovered from its own
optimization, floored by
`mapfusion.interRobot.map_alignment_{rotation,translation}_stddev_*`, and
propagated. The alignment and the registrations remain correlated and are
treated as independent, which errs towards a *larger* covariance; treating the
alignment as exact erred towards a smaller one. `test_skid_pose_uncertainty`
pins that direction as a property of the composition chain.

This matters only with three or more robots. With two, the same-peer branch
above applies and no alignment enters.

#### Blockers to representing remote keyframes

Full Equation (6) parity needs remote poses as variables in the local graph.
Three concrete things block that today, all verified against the current code:

1. `src/mapOptmization.cpp`, `correctPoses()`: iterates
   `numPoses = isamCurrentEstimate.size()` and indexes both
   `isamCurrentEstimate.at<Pose3>(i)` and `cloudKeyPoses3D->points[i]` by the
   same `i`. Any variable that is not a local keyframe breaks the
   correspondence and the key lookup.
2. `src/mapOptmization.cpp`, `saveKeyFramesAndFactor()`: takes the newest pose
   as `isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size() - 1)` and its
   covariance as `isam->marginalCovariance(isamCurrentEstimate.size() - 1)`.
   Both assume the highest key is the newest local keyframe.
3. `msg/LoopConstraint.msg` names one `robot_id` and two indices, so it cannot
   express endpoints belonging to two different trajectories.

The order to remove them in is 1 and 2 first -- give local keyframes an
explicit symbol space and iterate the keyframe count rather than the value
count -- then 3, then remote variables with priors supplied by the observing
robot. None of that is attempted here.

## Paper-to-package gaps

| Priority | Paper requirement | Current package | Required work |
|---|---|---|---|
| P0 | KISS-Matcher coarse registration (Section IV-C, Equation 9) | Implemented with pinned KISS-Matcher, Faster-PFH matching, ROBIN pruning, and robust estimation. | Add bag-level recall/timing evaluation. |
| P0 | Small-GICP fine registration (Figure 2 and Section IV-C) | Implemented with the KISS result as the initial estimate and one tested `target <- source` convention. | Add field-dataset accuracy evaluation. |
| P0 | KISS correspondence sanity check | Implemented with correspondence, solver-inlier, finite/rigid-transform, and exception gates. | Add bag-level rejection diagnostics. |
| P0 | Truncated MSE measurement gate (Section IV-D.1, Equation 10) | Implemented in squared metres with separate overlap/inlier gates and exact tests. | Calibrate thresholds per dataset. |
| P0 | Delivery of accepted loop factors | Implemented with one parameter-derived topic and typed `LoopConstraint` publisher/subscriber contract. | Add a launch-level two-node delivery test. |
| P1 | One SOLiD/registration pipeline for inter- and intra-robot loops | Implemented. Intra-robot loops are detected with SOLiD and registered and gated by the same module map fusion uses; the descriptor distance and the registration parameter set are single-sourced. Map fusion still skips same-robot candidates, which is now the correct division of work rather than a gap. | Calibrate the intra-robot gates on field data and add a bag-level comparison against the Scan Context baseline. |
| P1 | Distributed keyframe PGO matching Equation 6 | Partial approximation, now analysed rather than assumed. Map fusion still optimizes one `Pose3` per robot/map while each robot owns a separate keyframe graph. The two-robot case is shown below to lose no information; the multi-peer case no longer treats the map alignment as exact. | Represent remote keyframe variables explicitly. The three code-level blockers are named in "Equation (6) and the two-level formulation" below. |
| P1 | Lightweight message pool | Implemented. Announcements are descriptor-only; scans transfer on request. Announcement backlogs, the scan cache, outstanding requests, and parked candidates are all bounded, with defined retry, backpressure, and abandonment behaviour, and byte/latency reporting on both channels. | Measure the achieved bandwidth and latency on field bags and calibrate the cache budget against the datasets' revisit horizons. |
| P1 | Meaningful inter-robot uncertainty | Implemented for the SOLiD paper pipeline: Hessian-shaped, physically calibrated full covariance is propagated through PCM, map alignment, the typed loop message, and the GTSAM factor. | Calibrate on field data and extend trajectory uncertainty beyond the configured PCM floor. |
| P2 | ROS + ZeroMQ field communication setup (Section VI-B) | No ZeroMQ transport or reproducible network setup is present. The message contract is now transport-agnostic: announcements and scans are separate, bounded topics, so a bridge carries descriptors and scans independently. | Add an optional transport adapter or document the external component used by the paper. |
| P2 | Paper dataset configurations | Implemented. GEODE, GRACO, Majang, Moon, Park, and STEAM are ported to ROS 2 parameters with a launch file each, and every parameter file is checked against the declared parameter contract by `validate_config_parameters`. | Verify each against its bag: topic names, `imuRate`, and the Moon sensor/topic pairing noted in that file are unverified against real data. Add cave and planetary field configs if the data are available. |
| P2 | Paper evaluation harness | No reproducible PR, RTE/RRE, success-rate, ATE/ARE, descriptor-memory, or communication-latency pipeline is included. | Add scripts, manifests, ground-truth conventions, and expected result summaries. |
| P2 | Pipeline test coverage | Unit coverage now includes coarse-to-fine registration, truncated MSE, covariance, SE(3) PCM propagation and inversion, message conversion, SOLiD descriptor construction, descriptor retrieval, the shared registration parameter contract, the communication policy, and a parameter-contract check over every configuration and launch file. | Add failure injection and multi-robot graph tests. |
| P2 | Public package identity | The ROS package is still named and described as Liorf and retains upstream maintainer metadata. | Decide whether to rename the ROS package; at minimum correct description, authorship, dependencies, and third-party notices. |

## Current implementation evidence

- `src/skid_registration.cpp`: KISS-Matcher, Small-GICP, Equation (10), and
  Hessian-shaped covariance are isolated from ROS and unit tested.
- `src/skid_pose_uncertainty.cpp`: first-order SE(3) covariance propagation and
  the covariance-weighted Equation (11) residual are shared testable code.
- `src/liorf-DiSO/mapFusion_so.cpp`, `registerRelativeMotion()`: the paper
  coarse-to-fine pipeline produces a pose and full covariance.
- `msg/LoopConstraint.msg` and `include/loop_constraint_utils.hpp`: accepted
  factors no longer overload descriptor dimensions or a scalar intensity.
- `src/mapOptmization.cpp`, `contextLoopInfoHandler()`: the per-platform graph
  consumes the full covariance through `noiseModel::Gaussian::Covariance`.
- `include/skid_loop_detection.hpp`, `src/skid_loop_detection.cpp`: the SOLiD
  distance, circular-shift yaw, and candidate rules used by both nodes.
- `include/skid_solid_descriptor.hpp`, `src/skid_solid_descriptor.cpp`: the
  descriptor adapter that keeps SOLiD's global `PointType`/`PointXYZIRPYT`
  declarations out of the mapping node.
- `include/skid_registration_params.hpp`: the one registration parameter set
  both nodes declare and validate.
- `src/mapOptmization.cpp`, `registerLoopKeyframes()`, `addIntraRobotLoop()`,
  `performSOLiDLoopClosure()`: intra-robot detection, registration, gating, and
  covariance-weighted factor construction.
- `src/liorf-DiSO/mapFusion_so.cpp`, `KNNSearch()`: candidates from the same
  robot are discarded, because the local node now owns them.
- `include/skid_comms.hpp`, `src/skid_comms.cpp`: the bounded announcement
  queue, scan-cache retention policy, request tracker, deferred-candidate
  store, and transfer accounting.
- `msg/ScanRequest.msg`, `msg/ScanData.msg`: the on-demand scan channel.
- `src/liorf-DiSO/mapFusion_so.cpp`, `publishContextInfo()`: announcements
  carry the descriptor; the scan is omitted unless
  `mapfusion.comms.announce_scans` is set.
- `src/liorf-DiSO/mapFusion_so.cpp`, `ensureCandidateScans()`,
  `scanDataHandler()`: candidates park until the scans they need arrive.
- `src/liorf-DiSO/mapFusion_so.cpp`, `gtsamFactorGraph()`: graph keys are robot
  numeric identifiers, not the keyframe states in the paper's Equation 6.
- `src/liorf-DiSO/mapFusion_so.cpp`, `gtsamExpressionGraph()`, `mapAlignment()`,
  `sendLoopThis()`: the map alignment's marginal is recovered, floored, and
  propagated into cross-peer loop factors.
- `config/{geode,graco,majang_rover,moon,park,steam_legged}.yaml` and their
  launch files: the paper's field datasets, ported from `0611f71`.
- `test/validate_config_parameters.py`: every parameter file is checked
  against the parameters the sources declare, including the prefixed
  registration sets built at runtime, and against the declared types.
- `third_party/` and `CMakeLists.txt`: KISS-Matcher, Small-GICP, ROBIN, PMC,
  and xenium are pinned and built locally. ZeroMQ remains absent.

## Recoverable paper-era material

Use the following paths through `git show 0611f71:<path>` as references:

- `CMakeLists.txt`: KISS-Matcher, Small-GICP, TEASER++, ROBIN, xenium, and TBB
  dependency declarations.
- `src/Distributed-SOLiD-SLAM/mapFusion.cpp`: KISS correspondence generation,
  robust coarse estimation, Small-GICP refinement, and truncated fitness.
- `config/params_geode.yaml`
- `config/params_graco.yaml`
- `config/params_majang_rover.yaml`
- `config/params_moon.yaml`
- `config/params_park.yaml`
- `config/params_steam_legged.yaml`
- `src/featureExtraction.cpp`: the paper-era LIO-SAM feature front end.

Known defects in that historical prototype include:

- PCL KD-tree nearest-neighbor distances are already squared, but the old
  truncated-fitness helper squares them again.
- The helper normalizes once internally and again by the Small-GICP inlier
  count.
- Registration has no robust handling for too few KISS correspondences,
  non-finite estimates, or a failed fine-registration result.
- Registration constants are hard-coded per function instead of declared as
  validated ROS parameters.
- It is ROS 1 code and assumes the old single-map TF contract.

## Implementation plan for `v3`

### Phase 1: paper registration core

Completed:

1. Pin KISS-Matcher and Small-GICP as nested Git submodules.
2. Wire both dependencies into CMake without downloading during a normal
   configure/build.
3. Implement a ROS-independent registration library with explicit result and
   rejection status types.
4. Implement and unit-test truncated MSE independently of either solver.
5. Test known rigid transforms, large yaw offsets, partial overlap, outliers,
   empty clouds, insufficient correspondences, and non-convergence.

### Phase 2: map-fusion integration

Items 1-3 are complete; item 4 is partially complete:

1. Replace `icpRelativeMotion()` with coarse-to-fine registration.
2. Expose voxel sizes, radii, correspondence gates, fine-registration limits,
   and truncated-MSE thresholds as validated parameters.
3. Repair the accepted-loop topic contract.
4. Registration publishes correspondences, solver status, overlap, truncated
   MSE, uncertainty, and timing. Candidate-level and PCM acceptance/rejection
   counters still need a structured diagnostic topic.

### Phase 3: inter/intra parity and graph semantics

1. **Completed:** the same descriptor, registration, gating, and covariance
   code serves intra-robot loops. See "Intra-robot loop closure" below for the
   division of work that remains between the two nodes.
2. **Completed for factor delivery:** introduce a typed loop-factor message
   rather than overloading `ContextInfo` dimensions and a scalar score. Direct
   cross-robot state semantics still belong to item 3.
3. Make the optimized state and information matrices match Equation 6, or
   provide a documented equivalence proof and evaluation. **Partially done:**
   the analysis and the exact departures are recorded in "Equation (6) and the
   two-level formulation"; the multi-peer information matrices are corrected.
   Remote keyframe variables are still not represented.

### Phase 4: resource and evaluation parity

1. **Completed:** split descriptor announcements from requested scan payloads.
2. **Completed:** bound all queues and scan caches.
3. **Partially done:** the six paper field configurations are restored with
   launches and a parameter-contract test. Dataset manifests and ground-truth
   conventions still belong to item 4.
4. Reproduce the paper's place-recognition, registration, mapping, memory, and
   communication metrics.

## Phase 1 acceptance criteria

- A clean recursive clone configures without unpinned network downloads.
- Missing nested dependencies fail configure with an actionable message.
- Synthetic coarse-to-fine registration recovers large initial rotation and
  translation within declared tolerances.
- Fine registration uses the coarse transform as its initial estimate.
- Truncated MSE is correct in squared metres and remains useful under partial
  overlap.
- Invalid or weak coarse matches never reach PCM or the factor graph.
- ROS 2 Lyrical builds and all existing tests continue to pass.
