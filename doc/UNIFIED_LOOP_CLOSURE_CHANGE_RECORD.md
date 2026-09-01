# Change Record: Unified Intra- and Inter-Robot Loop Closure

Date: 1 September 2026

Branch: `claude/v3-paper-parity-wha23w`

Baseline: commit `d205d92`, *Propagate registration uncertainty through loop
factors*

Scope: the `v3` paper-parity item recorded in
[`PAPER_V3_GAP_AUDIT.md`](PAPER_V3_GAP_AUDIT.md) as
*P1 — One SOLiD/registration pipeline for inter- and intra-robot loops*, and
the Phase 3 item 1 that depended on it.

This document records what changed, why each decision was made, and exactly
how far the work was verified. It is a change record, not a design
specification; the pipeline itself is described in the audit's *Intra-robot
loop closure* section.

______________________________________________________________________

## 1. Problem

The paper describes one place-recognition and registration pipeline. The
package ran two.

The map-fusion node (`mapFusion_so.cpp`) already used the paper pipeline for
inter-robot loops: SOLiD retrieval, KISS-Matcher coarse registration,
Small-GICP fine registration, the Equation (10) truncated-MSE gate, and a
Hessian-shaped 6-DoF covariance carried into PCM and the factor graph.

The local mapping node (`mapOptmization.cpp`) used something else entirely for
intra-robot loops:

| Stage | Radius-search path | Scan Context path |
|---|---|---|
| Retrieval | KD-tree over keyframe positions | Scan Context ring/sector keys |
| Registration | PCL `GeneralizedIterativeClosestPoint` | PCL `IterativeClosestPoint` |
| Gate | `getFitnessScore() > historyKeyframeFitnessScore` | same scalar gate |
| Factor weight | isotropic diagonal from the fitness score | fixed 0.5 m² isotropic, Cauchy-wrapped |

Neither path produced a real covariance, and neither shared code, parameters,
or gates with the inter-robot path. Two descriptors, two registration
back ends, and two notions of "good enough" coexisted in one package.

## 2. What changed

### 2.1 New shared components

Three new units hold what both nodes now share. All three are free of ROS, and
two are free of PCL, so they are unit-testable without a ROS installation.

| Unit | Files | Responsibility |
|---|---|---|
| `liorf::loop_detection` | `include/skid_loop_detection.hpp`, `src/skid_loop_detection.cpp` | SOLiD descriptor distance, the circular A-SOLiD shift, the yaw that shift implies, and a descriptor index with revisit-exclusion rules |
| `liorf::solid` | `include/skid_solid_descriptor.hpp`, `src/skid_solid_descriptor.cpp` | Descriptor construction over the vendored SOLiD implementation |
| `liorf::registration::declareConfig` | `include/skid_registration_params.hpp` | The registration parameter set and its validation, declared once for both nodes |

`skid_solid_descriptor` exists for a specific reason: `SOLiD/solid.h` declares
global `PointType` and `PointXYZIRPYT` symbols that collide with the mapping
node's own definitions. Confining that header to one translation unit is what
makes the descriptor reachable from `mapOptmization.cpp` at all.

### 2.2 Local mapping node

`src/mapOptmization.cpp`:

- `performSCLoopClosure()` is replaced by `performSOLiDLoopClosure()`, which
  retrieves revisit candidates from the shared descriptor index.
- `performRSLoopClosure()` keeps its radius-search and external-trigger
  detection but no longer performs its own alignment.
- Both now call `addIntraRobotLoop()`, which calls `registerLoopKeyframes()`:
  coarse-to-fine registration, the paper gates, first-order covariance
  propagation through the SE(3) cycle, and a covariance-weighted factor.
- `loopNoiseModel()` builds that factor's weight from the propagated
  covariance, optionally behind a Cauchy kernel.
- `declareLoopClosureParameters()` declares and validates the new
  configuration at construction, so a bad parameter file fails at startup with
  a named reason rather than silently degrading at run time.
- Each keyframe's deskewed scan is described with SOLiD and appended to the
  index in `saveKeyFramesAndFactor()`.
- `SCManager`, `SCInputType`, and every PCL ICP/GICP loop registration are
  gone from this node.

### 2.3 Map-fusion node

`src/liorf-DiSO/mapFusion_so.cpp`:

- `distBtnSOLiDs()` now delegates to `liorf::loop_detection::compare()`. The
  duplicated cosine-distance and circular-L1 implementation is gone, and an
  unusable descriptor pair now returns infinity rather than a NaN that would
  compare false against every gate.
- The coarse yaw guess uses `liorf::loop_detection::sectorShiftToYaw()`.
- A 70-line block of `_registration_config.* = declare_and_get<...>(...)`
  statements plus its separate validation call is replaced by one
  `declareConfig()` call.

### 2.4 Build, configuration, and documentation

- `CMakeLists.txt`: adds the `skid_loop_detection` and
  `skid_solid_descriptor` libraries, links them plus `skid_registration` into
  `liorf_mapOptmization`, links `skid_loop_detection` into `liorf_mapFusion`,
  registers three new test targets, and drops `include/Scancontext.cpp` from
  the mapping executable's sources.
- `config/lio_sam_default.yaml`: documents the new `liorf.loopClosure.*`
  block. The other dataset configs are unchanged and fall back to defaults.
- `src/liorf-DiSO/third_parties/SOLiD/solid.h`: drops an unused
  `<pcl_conversions/pcl_conversions.h>` include. Neither `solid.h` nor
  `solid.cpp` uses a ROS type; removing it is what allows the descriptor to be
  built and tested outside ROS. Both call sites already include
  `pcl_conversions` directly, so nothing loses it.
- `README.md` and `PAPER_V3_GAP_AUDIT.md` record the new state.

### 2.5 Files retained

`include/Scancontext.h`, `include/Scancontext.cpp`, `include/nanoflann.hpp`,
`include/KDTreeVectorOfVectorsAdaptor.h`, and `include/tictoc.h` are no longer
referenced by any build target once the mapping node stops using Scan Context.
They were removed in commit `680e941` and **restored** afterwards: they stay in
the tree for reference and are simply not compiled.

The Scan Context baseline itself was never at risk. `liorf_mapFusionSC` builds
from an independent copy under `src/liorf-DiSO/third_parties/scanContext`.

## 3. Design decisions

Each of these was a real choice with a plausible alternative, so the reasoning
is recorded rather than left in the diff.

### 3.1 Map-frame clouds, not keyframe-local clouds

`registerLoopKeyframes()` expresses both clouds in the map frame using the
current, still drifted keyframe poses, exactly as the previous radius-search
path did. The alternative — registering in each keyframe's own frame to get
the relative pose directly — is cleaner in principle but changes the frame
contract of a path that already worked.

KISS-Matcher takes no initial guess, so drift only has to be small enough for
the two scans to overlap. Keeping the existing convention also means the
correction composes with the drifted pose and differences against the target
pose through the same `liorf::uncertainty::compose` / `between` sequence the
map-fusion node already uses, rather than a second, differently ordered one.

### 3.2 Cauchy kernel on by default for intra-robot factors

Inter-robot loops pass through pairwise consistency maximization before they
reach a factor graph. Intra-robot loops have no equivalent screen, and the
Scan Context path they replace deliberately used a Cauchy kernel for that
reason.

Dropping the kernel while introducing a new detector would have removed
outlier protection at the moment it was most needed. The kernel is enabled by
default and is configurable (`liorf.loopClosure.robustKernel`). It composes
cleanly with the covariance work rather than undoing it: the registration
covariance still sets the factor's shape and scale, and the kernel only bounds
how far a gross outlier can pull the graph.

### 3.3 Descriptor parameters shared under `mapfusion.solid`

The mapping node reads its SOLiD parameters from `mapfusion.solid.*` and its
descriptor acceptance distance from `mapfusion.interRobot.loop_threshold`,
rather than declaring a private copy.

The namespace reads oddly for an intra-robot path, but the alternative is
worse: two independently tunable descriptor configurations would let a place
be described one way locally and another way when exchanged, which is exactly
the divergence this work removes. Every launch file already passes
`mapfusion_solid.yaml` to every node, so the values reach both.

### 3.4 Truncated-MSE gate defaults to `historyKeyframeFitnessScore`

Intra-robot `max_truncated_mse_m2` defaults to the existing
`historyKeyframeFitnessScore` rather than to the inter-robot default of 3.0.

Both are mean squared nearest-neighbour distances in m². PCL's is untruncated
and therefore never smaller than the paper's truncated metric, so a threshold
tuned against PCL is a permissive bound for the new gate, not a tighter one —
the safe direction. This preserves per-dataset tuning that already exists
instead of orphaning it, and it is overridable per node. Per-dataset
calibration remains outstanding either way.

### 3.5 Exact linear retrieval in the mapping node

The map-fusion node keeps its libnabo KD-tree; the mapping node uses an exact
linear scan over its own keyframes.

Both rank by Euclidean distance between L2-normalized R-SOLiD keys, which is
monotonic in the cosine distance the shared comparison reports, so the two
order candidates identically — the shortlist only bounds how many A-SOLiD
circular alignments are evaluated. Retrieval is linear in keyframe count,
which at loop-closure rates costs far less than the registration it feeds.
Rewriting map fusion's multi-robot KD-tree bookkeeping to share a structure
would have been risk without benefit.

### 3.6 Yaw off-by-one corrected

The coarse yaw derived from the winning A-SOLiD shift used `(shift + 1)`
sector widths. Sector index increases with azimuth and the shift carries query
sector `i` onto candidate sector `i + shift`, so the correct value is `shift`
sector widths — the previous expression added one extra sector (6° at the
default 60 sectors) to every coarse guess, and could never return zero yaw.

This value is only a pre-rotation applied before an initial-guess-free global
registration, so the practical effect is small. It was nonetheless wrong, is
now correct in the one shared helper, and is covered by tests.

## 4. New parameters

All are optional; every one has a working default.

### `liorf.loopClosure.*` (local mapping node)

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `enableRadiusSearch` | bool | `true` | Detect revisits by radius search over the trajectory |
| `enableSolidSearch` | bool | `true` | Detect revisits with the SOLiD descriptor |
| `solidMinPoints` | int | `100` | Scans smaller than this cannot describe a place |
| `minKeyframeGap` | int | `30` | Descriptor entries; a nearer match is the current place, not a revisit |
| `minTimeGap` | double | `historyKeyframeSearchTimeDiff` | Seconds a revisit must clear |
| `robustKernel` | bool | `true` | Wrap the loop factor in a Cauchy kernel |
| `robustKernelScale` | double | `1.0` | Cauchy `k`; must be finite and positive |
| `registration.*` | — | shared defaults | 29 parameters, identical in name, meaning, and units to `mapfusion.registration.*` |

`registration.max_truncated_mse_m2` is the one parameter whose default differs
between the two nodes: `historyKeyframeFitnessScore` locally, and
`mapfusion.interRobot.icp_threshold` in map fusion.

### Reused, not new

`mapfusion.solid.max_range`, `knn_feature_dim`, `num_sector`, `num_height`,
`fov_up`, `fov_down`, `num_nearest_matches`, `num_match_candidates`, and
`mapfusion.interRobot.loop_threshold` are now read by both nodes.

## 5. Behavioural changes for operators

- Intra-robot loop registration is now global (KISS-Matcher) rather than
  local (ICP). It is more robust to drift and more expensive per attempt.
  Sites running a high `loopClosureFrequency` on constrained hardware should
  watch the loop thread's timing; per-stage timings are logged on every
  accepted loop.
- Accepted loops log at `INFO` with the detector name, keyframe pair,
  truncated MSE, overlap, and inlier count. Rejections log at `DEBUG` with the
  rejection status and the gate values that produced it — enable debug logging
  on `liorf_mapOptmization` when tuning thresholds.
- Loop factors are now full 6×6 Gaussians, not isotropic diagonals. A loop
  whose geometry is weakly observable in one direction is correctly weighted
  as such rather than pulling the graph equally in all six degrees of freedom.
- Scan Context no longer influences the local pose graph. Runs are not
  bit-comparable with the previous behaviour, and the Scan Context baseline
  for comparison is the separate `liorf_mapFusionSC` node.

## 6. Verification

### What was built and run

No ROS 2 installation was available in the working environment. The pinned
solver submodules were initialized and the ROS-independent code was built and
tested against Eigen 3.4, PCL 1.14, TBB, and the pinned
KISS-Matcher / Small-GICP / ROBIN / PMC / xenium revisions.

| Suite | Result | Covers |
|---|---|---|
| `test_skid_registration` | pass | Existing coarse-to-fine pipeline, including synthetic recovery of a 2.1-radian rigid rotation |
| `test_skid_loop_detection` | pass | 12 cases: configuration validation, sector-shift-to-yaw wrapping, descriptor comparison, unusable-descriptor rejection, index rejection rules, revisit exclusion, ranking, threshold, candidate budget, clear |
| `test_skid_solid_descriptor` | pass | 6 cases: parameter validation, descriptor dimensions, rejection of null/sparse/contrast-free scans, yaw invariance of R-SOLiD with a correctly shifted A-SOLiD, and end-to-end retrieval of a rotated revisit |
| `test_skid_registration_params` | pass | 5 cases: defaults match the documented configuration, all 29 parameters declared under the prefix, overrides applied, negative counts rejected, invalid values rejected by the shared validator |
| `test_observable_scan_match` | pass | Pre-existing, unaffected — confirms no regression |

All new and changed non-ROS sources compile clean under
`-Wall -Wextra -Wpedantic`. The generic-lambda-calling-a-member-template
pattern used by `declareConfig()` was separately compile-checked against both
node shapes (public member on a base class, private member on the class
itself), since it is the least conventional construct introduced.

Two defects were found and fixed by these tests during development rather than
by inspection: an incorrect expectation about shift wrapping that exposed the
need to state the yaw direction unambiguously, and a rotationally symmetric
synthetic scene that would have made the descriptor shift test vacuous.

### What was not verified

**The ROS nodes were not compiled or run.** `mapOptmization.cpp` and
`mapFusion_so.cpp` require GTSAM and `rclcpp`, neither of which was
installable in this environment. Both diffs were reviewed line by line, and
the non-ROS units they depend on are tested, but that is review, not a build.

Before the intra-robot claims in the audit can be called measured rather than
implemented, the following are still required:

1. A ROS 2 Lyrical `colcon build` of all executables and the full test suite.
2. A bag replay confirming that intra-robot loops are detected, registered,
   accepted, and applied to the pose graph.
3. Per-dataset calibration of `max_truncated_mse_m2`, `min_overlap_ratio`, and
   the SOLiD `loop_threshold` against ground truth.
4. A comparison against the Scan Context baseline on the same bags, to confirm
   the new detector's recall is at least as good.

## 7. Follow-up work

Unchanged by this session and still open in the audit:

- **P1** — Equation (6) keyframe-state semantics in the map-level graph.
- **P1** — Bounded, on-demand inter-robot communications.
- **P2** — Paper dataset configurations (GEODE, GRACO, Majang, Moon, Park, STEAM).
- **P2** — ZeroMQ field-communication transport or documentation.
- **P2** — The paper evaluation harness.

Newly opened by this session:

- Calibrate the intra-robot gates on field data.
- Add a bag-level comparison of SOLiD against the Scan Context baseline.
- Add topic/launch routing, failure-injection, and multi-robot graph tests.

## 8. Commits

| Commit | Contents |
|---|---|
| `680e941` | Unify intra- and inter-robot loop closure on the paper pipeline |
| `b3aa857` | Restore the retained Scan Context and vendored headers; add this change record |
