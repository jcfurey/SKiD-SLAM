# SKiD-SLAM Paper v3 Parity Audit

Status: living audit and implementation record for the `v3` branch

Last updated: 2 September 2026 (complete synthetic two-robot factor audit,
corrected PCM direction, delayed factor commitment, communication routing,
and TF timestamp provenance; previous registration, evaluation, graph, and
frame work retained)

Audit baseline: commit `475b59f`, before the paper-registration work below.
The gap table records that baseline so the provenance problem remains visible;
the implementation-status section records what has since been restored.

Paper: `2505.08230v3`, *SKiD-SLAM: Robust, Lightweight, and
Distributed Multi-Robot LiDAR SLAM in Resource-Constrained Field
Environments* (30 July 2025)

## Change records

[`SESSION_CHANGE_LOG.md`](SESSION_CHANGE_LOG.md) indexes the 1 September 2026
session as a whole, including the defects it found and the verification
boundary for all of it.

Per-change records, each with the decisions behind it and its verification
boundary:

- [`UNIFIED_LOOP_CLOSURE_CHANGE_RECORD.md`](UNIFIED_LOOP_CLOSURE_CHANGE_RECORD.md)
- [`BOUNDED_COMMUNICATIONS_CHANGE_RECORD.md`](BOUNDED_COMMUNICATIONS_CHANGE_RECORD.md)
- [`MAP_ALIGNMENT_UNCERTAINTY_CHANGE_RECORD.md`](MAP_ALIGNMENT_UNCERTAINTY_CHANGE_RECORD.md)
- [`DISTRIBUTED_KEYFRAME_GRAPH_CHANGE_RECORD.md`](DISTRIBUTED_KEYFRAME_GRAPH_CHANGE_RECORD.md)
- [`PCM_COMMITMENT_CHANGE_RECORD.md`](PCM_COMMITMENT_CHANGE_RECORD.md)
- [`FULL_TWO_ROBOT_REPLAY_CHANGE_RECORD.md`](FULL_TWO_ROBOT_REPLAY_CHANGE_RECORD.md)
- [`EVALUATION_HARNESS_CHANGE_RECORD.md`](EVALUATION_HARNESS_CHANGE_RECORD.md)
- [`FIELD_COMMUNICATION_CHANGE_RECORD.md`](FIELD_COMMUNICATION_CHANGE_RECORD.md)

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
- Per-robot GTSAM pose graphs augmented by sparse remote keyframe states and
  direct PCM-committed cross-robot factors.
- A separate map-level inter-robot alignment graph for fleet TF placement.
- ROS 2 Jazzy/Lyrical build and launch support.
- RESPLE/X-ICP-inspired observable-subspace local scan matching.
- REP-105 local frame separation and optional ECEF geographic anchoring.
- Descriptor-only announcements with on-demand scan transfer, bounded queues
  and caches, and byte/latency reporting.
- The six paper dataset configurations, with a parameter-contract test.
- A dependency-free evaluation harness for the paper's metrics.
- A ZeroMQ peer transport and inter-robot bridge for field deployment.

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
- PCM receives `target <- source` measurements as Equation (11) requires; the
  legacy registration queue's stored `source <- target` pose is inverted with
  covariance propagation before the consistency cycle is evaluated.
- Current maximum-clique membership is provisional. A configurable minimum
  clique size and consecutive-membership count produce a monotonic committed
  set, and only that set may affect map alignment or either factor publisher.
  A factor-publication switch supports non-mutating calibration runs.
- Accepted loops use `LoopConstraint.msg`, with a quaternion pose, full 6x6
  covariance, explicit robot/keyframe endpoints, owner-frame endpoint poses,
  and registration diagnostics. The mapping back end consumes the covariance
  as a full GTSAM Gaussian model.
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
- PCM-committed inter-robot registrations are now delivered directly to both
  endpoint robots. Each local optimizer retains its own `X(index)` variables
  plus the Equation (7) subset of peer keyframes, joined by sparse peer-motion
  edges. Map alignment remains a TF/coordination output and no longer mediates
  these graph factors in the default mode.

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

The intra-robot pipeline reuse was first verified in isolation, then built as a
complete ROS 2 Lyrical package on 1 September 2026. A full 161.3-second
HelmDyn03 replay at 1x delivered every LiDAR and IMU input, produced odometry at
about 20 Hz, and exercised accepted radius-search and SOLiD intra-robot loops.
That established a sustained single-robot smoke test.

On 2 September, a corrected factor-enabled 70-second segment of the synthetic
HelmDyn08/09 fixture ran two estimator/map-fusion pipelines at 2x. Each bridge
forwarded 1,400 LiDAR and 14,001 IMU messages without a reported loss or
mismatch. The trace contained 8,566 descriptor candidates, 86 accepted
registrations, 23 commitment transitions, and 37 unique direct factors
recorded on each endpoint topic. The INFO console contained no KISS
noise-bound warnings: the explicit default effective bounds are now 1.0 m for
ROBIN and 0.75 m for the solver, and validation prevents silent clamp reliance.

The replay uncovered and then verified a keyframe provenance fix. A queued
descriptor announcement had inherited the node's latest cloud timestamp when
its background FIFO drained; it now preserves the timestamp captured with the
descriptor. Of the 37 factors in the corrected run, 30 had both endpoint times
within 30 ms of retimed mocap, and all 30 had translation RTE below 1.33 m
(0.16 m median, 0.40 m p90). The remaining seven lie in mocap gaps and are not
included in that strict result. This remains a bounded synthetic test rather
than a paper-comparable field evaluation.

A complete-timeline diagnostic capture subsequently preserved 111 factors on
each endpoint topic. The factor auditor found no missing, additional,
duplicate, differently oriented, or numerically different delivery, and no
endpoint timestamp conflicts. Ninety-six factors associate with both retimed
mocap trajectories within 30 ms: RTE is 0.145 m median and 0.306 m p90, with
two values above 2 m. One has a corresponding 2.735 m separation error and is
an unresolved geometry outlier. The orientation convention remains unresolved,
so the reported RRE distribution is diagnostic rather than a parity claim.

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

The structural Equation (6)/(7) path is now implemented and has
complete-timeline synthetic two-pipeline runtime evidence. Remaining P1/P2
work is fidelity and stronger evidence: revisioned peer corrections or peer
factor exchange, real simultaneous multi-robot evaluation, remote-motion
covariance calibration, ZeroMQ bench/radio results, and paper-protocol
evaluation.

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

### Equation (6) and the distributed keyframe graph

A file-by-file record of the implementation and its verification boundary is
in
[`DISTRIBUTED_KEYFRAME_GRAPH_CHANGE_RECORD.md`](DISTRIBUTED_KEYFRAME_GRAPH_CHANGE_RECORD.md).

The old implementation optimized two separate levels: each mapping node held
only its local keyframes, while map fusion estimated one alignment per robot.
It waited for two registrations, eliminated the peer frame algebraically, and
published a local-to-local factor. That route remains available for comparison
but is no longer the default.

The default now follows the state structure of paper Equations (6) and (7):

- A platform's locally owned poses occupy explicit `X(index)` keys.
- Every PCM-committed registration is published directly between its two
  robot/keyframe endpoints and is delivered to both endpoint robots.
- Each recipient creates a distinct remote variable for the other endpoint.
  Only remote poses touched by accepted registrations are retained.
- Remote poses belonging to one peer are connected by a sparse spanning tree
  of owner-frame relative motions. Those differences are invariant to the
  peer map's unknown placement.
- Registration factors use the full propagated covariance. Remote-motion
  factors use separately configured angular and translational uncertainty
  floors.

Initial values for new remote poses are derived from the local endpoint and
the registration, but are not added as priors. Thus one direct factor fixes a
new remote state relative to a local pose without changing that local pose; a
second direct factor and peer-motion edge create the correction cycle. This is
the correct gauge behaviour and is pinned by a solver-level two-robot test.

Map fusion still estimates and publishes fleet-map-to-platform-map transforms,
but that alignment no longer mediates direct keyframe factors. This keeps the
Equation (6) graph independent of Earth, UTM/UPS, and potentially distant map
origins. Endpoint-pair identities are canonicalized so two observers reporting
the same registration in opposite directions cannot double-count it.

This is **structural parity**, with important fidelity limits. Peer odometry is
currently reconstructed from snapshots of announced optimized poses rather
than imported as its original factors and covariance; later corrections at the
owner do not revise stored remote edges; peer GPS and intra-robot loop factors
are not imported; and a committed factor cannot yet be retracted if a
persistent later clique excludes it. Delayed commitment now filters transient
and undersized cliques but does not replace factor removal. The default
uncertainty floors and field thresholds remain to be calibrated. These are now
the Equation (6) gaps—not absent symbol spaces, indexing assumptions, or an
endpoint-incapable message.

## Paper-to-package gaps

| Priority | Paper requirement | Current package | Required work |
|---|---|---|---|
| P0 | KISS-Matcher coarse registration (Section IV-C, Equation 9) | Implemented with pinned KISS-Matcher, Faster-PFH matching, ROBIN pruning, and robust estimation. Effective default noise bounds are explicit, and validation prevents warning-producing implicit clamping. | Extend the bounded HelmDyn trace into bag-level recall/timing evaluation on real field data. |
| P0 | Small-GICP fine registration (Figure 2 and Section IV-C) | Implemented with the KISS result as the initial estimate and one tested `target <- source` convention. | Add field-dataset accuracy evaluation. |
| P0 | KISS correspondence sanity check | Implemented with correspondence, solver-inlier, finite/rigid-transform, and exception gates. Structured bag diagnostics retain every rejection status and detail, while expected weak-candidate failures are DEBUG rather than WARN. | Inspect rejection distributions and calibrate them on real multi-robot field runs. |
| P0 | Truncated MSE measurement gate (Section IV-D.1, Equation 10) | Implemented in squared metres with separate overlap/inlier gates and exact tests. | Calibrate thresholds per dataset. |
| P0 | Delivery of accepted loop factors | Implemented with one parameter-derived topic and an endpoint-aware typed `LoopConstraint` contract. A registration is published to both endpoint graphs only after repeated membership in a sufficiently large PCM clique; factor queues and the bridge are sized for clique bursts. A complete synthetic trace delivered the same 111 unique factors to both endpoints, verified measurement-for-measurement. | Validate on real simultaneous runs, and add acknowledgement/replay if disconnected peers must recover missed factors. |
| P1 | One SOLiD/registration pipeline for inter- and intra-robot loops | Implemented. Intra-robot loops are detected with SOLiD and registered and gated by the same module map fusion uses; the descriptor distance and the registration parameter set are single-sourced. Map fusion still skips same-robot candidates, which is now the correct division of work rather than a gap. | Calibrate the intra-robot gates on field data and add a bag-level comparison against the Scan Context baseline. |
| P1 | Distributed keyframe PGO matching Equation 6 | Structurally implemented. Each optimizer has explicit local symbols, sparse peer-keyframe variables, direct full-covariance cross-robot factors, and peer-motion edges; the map-alignment graph is no longer in the factor path. The live direct-factor route has complete-timeline synthetic two-robot evidence. | Exchange revisioned peer corrections or original peer factors/covariances, support committed-factor retraction, and validate on real multi-robot field runs. |
| P1 | Lightweight message pool | Implemented. Announcements are descriptor-only; scans transfer on request. Announcement backlogs, the scan cache, outstanding requests, and parked candidates are all bounded, with defined retry, backpressure, and abandonment behaviour, and byte/latency reporting on both channels. Enqueue and dequeue share the same one-producer routing rule, and a complete post-fix replay reported no backlog drops. | Measure the achieved bandwidth and latency on field bags and calibrate the cache budget against the datasets' revisit horizons. |
| P1 | Meaningful inter-robot uncertainty | Implemented for registration: Hessian-shaped, physically scaled full covariance is propagated through PCM, the typed loop message, and each direct GTSAM factor. Sparse remote-motion edges have separate angular/linear floors. | Calibrate on field data and replace fixed remote-motion floors with the peer trajectory's propagated covariance. |
| P2 | ROS + ZeroMQ field communication setup (Section VI-B) | Implemented and compiled on ROS 2 Lyrical. `liorf_zmqBridge` carries the inter-robot topics over a ZeroMQ PUB/SUB mesh through a type-agnostic generic pub/sub bridge; the transport underneath is tested over real sockets. The deployment is in `doc/FIELD_COMMUNICATION.md`. The bridge is optional: a system without ZeroMQ builds everything else. | Bench-verify two bridge nodes across isolated ROS domains, then measure over a real radio. |
| P2 | Paper dataset configurations | Implemented. GEODE, GRACO, Majang, Moon, Park, and STEAM are ported to ROS 2 parameters with a launch file each, and every parameter file is checked against the declared parameter contract by `validate_config_parameters`. | Verify each against its bag: topic names, `imuRate`, and the Moon sensor/topic pairing noted in that file are unverified against real data. Add cave and planetary field configs if the data are available. |
| P2 | Paper evaluation harness | Implemented in `evaluation/`: PR curves, RTE/RRE and success rate, ATE/ARE with rigid, Sim(3) or yaw alignment, descriptor memory against a Scan Context baseline, and communication cost read back from the map-fusion diagnostics. `LoopDiagnostic` records every eligible descriptor score, scan outcome, registration result, and provisional/committed PCM decision with original keyframe timestamps; the bag tools extract CSVs and audit graph-facing factors symmetrically against ground truth. Manifests exist for all six datasets, and the metrics and extraction rules are tested against analytically known cases. Timestamp and delivery conventions were checked over the complete synthetic two-robot fixture. | Record a real multi-robot run, resolve the HelmDyn body/LiDAR orientation convention, and fill in each manifest's `expected` block once the paper's protocol is confirmed to match. |
| P2 | Pipeline test coverage | Unit coverage now includes coarse-to-fine registration, truncated MSE, covariance, SE(3) PCM propagation and inversion, nonidentity PCM direction, delayed commitment policy, endpoint-aware message conversion, SOLiD descriptor construction/retrieval, graph key namespaces, sparse remote trajectories, factor orientation/deduplication, a solver-level two-robot correction graph, communication policy, and the full parameter contract. | Add ROS launch-level assertions around factor content, disconnect/replay failure injection, and measured real multi-robot evaluation. |
| P2 | Public package identity | The ROS package is still named and described as Liorf and retains upstream maintainer metadata. | Decide whether to rename the ROS package; at minimum correct description, authorship, dependencies, and third-party notices. |

## Current implementation evidence

- `src/skid_registration.cpp`: KISS-Matcher, Small-GICP, Equation (10), and
  Hessian-shaped covariance are isolated from ROS and unit tested.
- `src/skid_pose_uncertainty.cpp`: first-order SE(3) covariance propagation and
  the covariance-weighted Equation (11) residual are shared testable code.
- `src/liorf-DiSO/mapFusion_so.cpp`, `registerRelativeMotion()`: the paper
  coarse-to-fine pipeline produces a pose and full covariance.
- `msg/LoopConstraint.msg` and `include/loop_constraint_utils.hpp`: accepted
  factors name both robot/keyframe endpoints, carry owner-frame endpoint poses,
  and no longer overload descriptor dimensions or a scalar intensity.
- `msg/LoopDiagnostic.msg`, `src/liorf-DiSO/mapFusion_so.cpp`, and
  `evaluation/extract_diagnostics_from_bag.py`: descriptor retrieval, scan
  acquisition, registration, and PCM decisions retain one candidate identity
  and can be converted directly into the harness CSV inputs.
- `include/skid_graph_keys.hpp` and `include/skid_remote_graph.hpp`: local and
  peer variables occupy separate symbol spaces; sparse peer trajectories,
  oriented initialization, and canonical factor identity are testable outside
  ROS.
- `src/mapOptmization.cpp`, `contextLoopInfoHandler()`: the per-platform iSAM2
  graph inserts direct local-to-peer factors with full covariance and sparse
  peer-motion edges.
- `src/liorf-DiSO/mapFusion_so.cpp`, `publishAcceptedDirectFactors()`: every
  newly PCM-committed registration is sent to both endpoint robots without
  composing through a map alignment.
- `include/skid_pcm_commitment.hpp`: minimum clique support, consecutive
  membership, and monotonic publication authority are isolated from ROS and
  unit tested.
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
  carry the descriptor and its captured keyframe timestamp; the scan is
  omitted unless `mapfusion.comms.announce_scans` is set.
- `src/liorf-DiSO/mapFusion_so.cpp`, `ensureCandidateScans()`,
  `scanDataHandler()`: candidates park until the scans they need arrive.
- `src/liorf-DiSO/mapFusion_so.cpp`, `gtsamFactorGraph()` and
  `gtsamExpressionGraph()`: the separate robot/map alignment graph remains for
  fleet TF placement. It is not the Equation (6) keyframe graph and does not
  mediate factors while direct mode is enabled.
- `config/{geode,graco,majang_rover,moon,park,steam_legged}.yaml` and their
  launch files: the paper's field datasets, ported from `0611f71`.
- `include/skid_transport.hpp`, `src/skid_transport.cpp`: the ZeroMQ peer
  transport and the echo suppressor, tested over real sockets.
- `src/liorf-DiSO/zmqBridge.cpp` and `doc/FIELD_COMMUNICATION.md`: the bridge
  node and its deployment.
- `evaluation/`: the metric implementations, dataset manifests, CLI runner,
  graph-facing factor auditor, and the conventions the inputs must follow.
- `test/validate_config_parameters.py`: every parameter file is checked
  against the parameters the sources declare, including the prefixed
  registration sets built at runtime, and against the declared types.
- `third_party/` and `CMakeLists.txt`: KISS-Matcher, Small-GICP, ROBIN, PMC,
  and xenium are pinned and built locally.

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

Items 1-4 are complete:

1. Replace `icpRelativeMotion()` with coarse-to-fine registration.
2. Expose voxel sizes, radii, correspondence gates, fine-registration limits,
   and truncated-MSE thresholds as validated parameters.
3. Repair the accepted-loop topic contract.
4. `LoopDiagnostic` publishes every eligible descriptor score (selected or
   rejected), scan acquisition outcome, full registration status/quality/
   uncertainty/timing record, and both provisional and committed PCM state.
   Candidate identity includes both robots' keyframe indices and original
   timestamps so the evaluation inputs can be extracted directly from a bag.
5. Correct Equation (11) measurement direction, explicit matcher bounds,
   delayed publication commitment, non-mutating dry runs, and runtime-only PCM
   scratch storage are implemented and covered by the two-robot stabilization
   record.

### Phase 3: inter/intra parity and graph semantics

1. **Completed:** the same descriptor, registration, gating, and covariance
   code serves intra-robot loops. See "Intra-robot loop closure" below for the
   division of work that remains between the two nodes.
2. **Completed:** the typed loop-factor message names both robot/keyframe
   endpoints, carries owner-frame endpoint poses, and delivers one direct
   registration to both endpoint graphs.
3. **Structurally completed; fidelity and evaluation remain:** local and sparse
   remote keyframes coexist in each optimizer and direct factors use the
   registration covariance as Equation (6) requires. Peer motion currently
   comes from announced pose snapshots with fixed uncertainty floors rather
   than revisioned original peer factors. See "Equation (6) and the distributed
   keyframe graph" and its change record. The direct route has been exercised
   over the complete synthetic two-pipeline fixture; real field runs remain.

### Phase 4: resource and evaluation parity

1. **Completed:** split descriptor announcements from requested scan payloads.
2. **Completed:** bound all queues and scan caches.
3. **Partially done:** the six paper field configurations are restored with
   launches and a parameter-contract test. Dataset manifests and ground-truth
   conventions still belong to item 4.
4. **Partially done:** the harness, structured diagnostic topic, bag
   extractors, and graph-facing factor auditor exist and are tested. A complete
   synthetic two-robot trace checks timestamp and factor conventions, but no
   paper-protocol real multi-robot result or expected figure is recorded. See
   `evaluation/README.md`.

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
