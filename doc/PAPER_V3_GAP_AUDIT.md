# SKiD-SLAM Paper v3 Parity Audit

Status: audit baseline and implementation record for the `v3` branch

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
- SOLiD descriptor construction and KD-tree inter-robot candidate search.
- PCM consistency matrices and maximum-clique filtering.
- Per-robot GTSAM pose graphs and a map-level inter-robot alignment graph.
- ROS 2 Jazzy/Lyrical build and launch support.
- RESPLE/X-ICP-inspired observable-subspace local scan matching.
- REP-105 local frame separation and optional ECEF geographic anchoring.

The last two items are post-paper extensions and should be retained while
paper parity is restored.

## `v3` implementation status

The first paper-parity slice is implemented on top of the audit baseline:

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

Verification on ROS 2 Lyrical includes a clean package build, all existing
frame and observable-scan-match tests, exact truncated-MSE tests, invalid-input
tests, and a solver-level synthetic test recovering a 2.1-radian rigid
rotation through the complete KISS-Matcher-to-Small-GICP pipeline.

The remaining P1/P2 work is unchanged: intra-robot pipeline reuse, Equation
(6) graph semantics, bounded/on-demand communications, uncertainty modeling,
field configurations, and the paper evaluation harness.

## Paper-to-package gaps

| Priority | Paper requirement | Current package | Required work |
|---|---|---|---|
| P0 | KISS-Matcher coarse registration (Section IV-C, Equation 9) | Absent. SOLiD yaw or a prior map transform seeds PCL GICP directly. | Add a pinned KISS-Matcher dependency and an isolated coarse-registration stage using Faster-PFH, geometric suppression, and robust transformation estimation. |
| P0 | Small-GICP fine registration (Figure 2 and Section IV-C) | Absent. `mapFusion_so.cpp` uses `pcl::GeneralizedIterativeClosestPoint`. | Add a pinned Small-GICP dependency and refine the KISS result with Small-GICP. Compose transforms in the actual order in which they are applied. |
| P0 | KISS correspondence sanity check | Absent. No minimum surviving-correspondence or solver-validity gate is present. | Reject insufficient, non-finite, or invalid coarse solutions before local refinement. Publish rejection reasons. |
| P0 | Truncated MSE measurement gate (Section IV-D.1, Equation 10) | Absent. The untruncated PCL fitness score is thresholded. | Implement the paper metric with explicit squared-distance units, overlap/inlier counts, and tests for partial overlap. |
| P0 | Delivery of accepted loop factors | Broken by a topic contract mismatch. Map fusion publishes `<robot>/solid/loop_info`; mapping subscribes to `<robot>/context/loop_info`. | Define one parameter-derived topic and use it at both endpoints. Add a launch/integration test. |
| P1 | One SOLiD/registration pipeline for inter- and intra-robot loops | Missing. Map fusion explicitly skips same-robot candidates; local loops use Scan Context plus PCL ICP. | Generalize candidate ownership and reuse the paper registration/gating module for both loop types. |
| P1 | Distributed keyframe PGO matching Equation 6 | Partial approximation. Map fusion optimizes one `Pose3` per robot/map while each robot owns a separate keyframe graph. | Represent accepted inter-robot keyframe factors and the relevant remote trajectory subset explicitly, or document and validate an equivalent distributed formulation. |
| P1 | Lightweight message pool | A simple pair of unbounded per-peer vectors is used. Every descriptor message also embeds the complete feature cloud, and received clouds are retained indefinitely. | Separate descriptor announcements from on-demand scan transfer, bound queues/caches, define retry/backpressure behavior, and report bytes and latency. |
| P1 | Meaningful inter-robot uncertainty | ICP fitness is copied into all six pose variances; PCM uses an identity weighting. | Derive or configure dimensionally valid registration covariance/information and propagate it into PCM and PGO. |
| P2 | ROS + ZeroMQ field communication setup (Section VI-B) | No ZeroMQ transport or reproducible network setup is present. | Add an optional transport adapter or document the external component used by the paper. |
| P2 | Paper dataset configurations | GEODE, GRACO, Majang, Moon, Park, and STEAM configs from `0611f71` are absent. | Port them to ROS 2 parameters and add sensor/topic validation. Add cave and planetary field configs if the data are available. |
| P2 | Paper evaluation harness | No reproducible PR, RTE/RRE, success-rate, ATE/ARE, descriptor-memory, or communication-latency pipeline is included. | Add scripts, manifests, ground-truth conventions, and expected result summaries. |
| P2 | Pipeline test coverage | Existing tests cover observable scan matching and geographic frames only. | Add SOLiD, coarse registration, fine registration, truncated MSE, PCM, topic routing, and multi-robot graph tests. |
| P2 | Public package identity | The ROS package is still named and described as Liorf and retains upstream maintainer metadata. | Decide whether to rename the ROS package; at minimum correct description, authorship, dependencies, and third-party notices. |

## Current implementation evidence

- `src/liorf-DiSO/mapFusion_so.cpp`, `getInitialGuess()`: the initial estimate
  comes from SOLiD yaw or an already-known map transform.
- `src/liorf-DiSO/mapFusion_so.cpp`, `icpRelativeMotion()`: the sole
  inter-robot registration solver is PCL GICP.
- `src/liorf-DiSO/mapFusion_so.cpp`, `KNNSearch()`: candidates from the same
  robot are discarded.
- `src/liorf-DiSO/mapFusion_so.cpp`, `publishContextInfo()`: the entire feature
  cloud is serialized into every context message.
- `src/liorf-DiSO/mapFusion_so.cpp`, `gtsamFactorGraph()`: graph keys are robot
  numeric identifiers, not the keyframe states in the paper's Equation 6.
- `src/mapOptmization.cpp`, constructor: the accepted-loop subscriber is fixed
  to `<robot>/context/loop_info`.
- `src/liorf-DiSO/config/mapfusion_solid.yaml`: `solid_topic` is `solid`, making
  the corresponding publisher `<robot>/solid/loop_info`.
- `CMakeLists.txt` and `package.xml`: KISS-Matcher, Small-GICP, TEASER++/GNC,
  ROBIN, TBB, and ZeroMQ are not dependencies.

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

1. Pin KISS-Matcher and Small-GICP as nested Git submodules.
2. Wire both dependencies into CMake without downloading during a normal
   configure/build.
3. Implement a ROS-independent registration library with explicit result and
   rejection status types.
4. Implement and unit-test truncated MSE independently of either solver.
5. Test known rigid transforms, large yaw offsets, partial overlap, outliers,
   empty clouds, insufficient correspondences, and non-convergence.

### Phase 2: map-fusion integration

1. Replace `icpRelativeMotion()` with coarse-to-fine registration.
2. Expose voxel sizes, radii, correspondence gates, fine-registration limits,
   and truncated-MSE thresholds as validated parameters.
3. Repair the accepted-loop topic contract.
4. Publish diagnostics for candidates, correspondences, solver status,
   overlap, truncated MSE, PCM acceptance, and timing.

### Phase 3: inter/intra parity and graph semantics

1. Reuse the same descriptor/registration/gating code for intra-robot loops.
2. Introduce a typed inter-robot factor message rather than overloading
   `ContextInfo` dimensions and `CloudInfo.imu_available` as keyframe indices.
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
