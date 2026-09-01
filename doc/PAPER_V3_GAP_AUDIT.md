# SKiD-SLAM Paper v3 Parity Audit

Status: living audit and implementation record for the `v3` branch

Last updated: 1 September 2026 (intra-robot pipeline reuse)

Audit baseline: commit `475b59f`, before the paper-registration work below.
The gap table records that baseline so the provenance problem remains visible;
the implementation-status section records what has since been restored.

Paper: `2505.08230v3`, *SKiD-SLAM: Robust, Lightweight, and
Distributed Multi-Robot LiDAR SLAM in Resource-Constrained Field
Environments* (30 July 2025)

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

The remaining P1/P2 work is Equation (6) keyframe-state semantics,
bounded/on-demand communications, field configurations, ZeroMQ deployment
documentation, and the paper evaluation harness.

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

The truncated-MSE gate for intra-robot loops defaults to
`historyKeyframeFitnessScore`. Both are mean squared nearest-neighbor
distances in m^2; PCL's is untruncated and therefore never smaller, so a
threshold tuned against it is a permissive bound for the paper's metric rather
than a tighter one. Per-dataset calibration is still outstanding.

## Paper-to-package gaps

| Priority | Paper requirement | Current package | Required work |
|---|---|---|---|
| P0 | KISS-Matcher coarse registration (Section IV-C, Equation 9) | Implemented with pinned KISS-Matcher, Faster-PFH matching, ROBIN pruning, and robust estimation. | Add bag-level recall/timing evaluation. |
| P0 | Small-GICP fine registration (Figure 2 and Section IV-C) | Implemented with the KISS result as the initial estimate and one tested `target <- source` convention. | Add field-dataset accuracy evaluation. |
| P0 | KISS correspondence sanity check | Implemented with correspondence, solver-inlier, finite/rigid-transform, and exception gates. | Add bag-level rejection diagnostics. |
| P0 | Truncated MSE measurement gate (Section IV-D.1, Equation 10) | Implemented in squared metres with separate overlap/inlier gates and exact tests. | Calibrate thresholds per dataset. |
| P0 | Delivery of accepted loop factors | Implemented with one parameter-derived topic and typed `LoopConstraint` publisher/subscriber contract. | Add a launch-level two-node delivery test. |
| P1 | One SOLiD/registration pipeline for inter- and intra-robot loops | Implemented. Intra-robot loops are detected with SOLiD and registered and gated by the same module map fusion uses; the descriptor distance and the registration parameter set are single-sourced. Map fusion still skips same-robot candidates, which is now the correct division of work rather than a gap. | Calibrate the intra-robot gates on field data and add a bag-level comparison against the Scan Context baseline. |
| P1 | Distributed keyframe PGO matching Equation 6 | Partial approximation. The factor transport is now typed and covariance-aware, but map fusion still optimizes one `Pose3` per robot/map while each robot owns a separate keyframe graph. | Represent direct cross-robot keyframe factors and the relevant remote trajectory subset explicitly, or document and validate an equivalent distributed formulation. |
| P1 | Lightweight message pool | A simple pair of unbounded per-peer vectors is used. Every descriptor message also embeds the complete feature cloud, and received clouds are retained indefinitely. | Separate descriptor announcements from on-demand scan transfer, bound queues/caches, define retry/backpressure behavior, and report bytes and latency. |
| P1 | Meaningful inter-robot uncertainty | Implemented for the SOLiD paper pipeline: Hessian-shaped, physically calibrated full covariance is propagated through PCM, map alignment, the typed loop message, and the GTSAM factor. | Calibrate on field data and extend trajectory uncertainty beyond the configured PCM floor. |
| P2 | ROS + ZeroMQ field communication setup (Section VI-B) | No ZeroMQ transport or reproducible network setup is present. | Add an optional transport adapter or document the external component used by the paper. |
| P2 | Paper dataset configurations | GEODE, GRACO, Majang, Moon, Park, and STEAM configs from `0611f71` are absent. | Port them to ROS 2 parameters and add sensor/topic validation. Add cave and planetary field configs if the data are available. |
| P2 | Paper evaluation harness | No reproducible PR, RTE/RRE, success-rate, ATE/ARE, descriptor-memory, or communication-latency pipeline is included. | Add scripts, manifests, ground-truth conventions, and expected result summaries. |
| P2 | Pipeline test coverage | Unit coverage now includes coarse-to-fine registration, truncated MSE, covariance, SE(3) PCM propagation, message conversion, SOLiD descriptor construction, descriptor retrieval, and the shared registration parameter contract, in addition to frame tests. | Add topic/launch routing, failure injection, and multi-robot graph tests. |
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
- `src/liorf-DiSO/mapFusion_so.cpp`, `publishContextInfo()`: the entire feature
  cloud is serialized into every context message.
- `src/liorf-DiSO/mapFusion_so.cpp`, `gtsamFactorGraph()`: graph keys are robot
  numeric identifiers, not the keyframe states in the paper's Equation 6.
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
   provide a documented equivalence proof and evaluation.

### Phase 4: resource and evaluation parity

1. Split descriptor announcements from requested scan payloads.
2. Bound all queues and scan caches.
3. Restore ROS 2 field configurations and dataset manifests.
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
