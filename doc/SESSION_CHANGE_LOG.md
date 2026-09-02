# Session Change Log

Session date: 1 September 2026

Original branch: `claude/v3-paper-parity-wha23w`; continued on `v3`

Session baseline: commit `d205d92`, *Propagate registration uncertainty through
loop factors*

This is the index for one working session on `v3` paper parity. Each slice has
its own change record with the decisions and the verification boundary behind
it; this document says what was done overall, what was found along the way, and
what the whole session leaves unverified.

## Change records

| Record | Audit item |
|---|---|
| [`UNIFIED_LOOP_CLOSURE_CHANGE_RECORD.md`](UNIFIED_LOOP_CLOSURE_CHANGE_RECORD.md) | P1 — One SOLiD/registration pipeline for inter- and intra-robot loops |
| [`BOUNDED_COMMUNICATIONS_CHANGE_RECORD.md`](BOUNDED_COMMUNICATIONS_CHANGE_RECORD.md) | P1 — Lightweight message pool |
| [`MAP_ALIGNMENT_UNCERTAINTY_CHANGE_RECORD.md`](MAP_ALIGNMENT_UNCERTAINTY_CHANGE_RECORD.md) | P1 — Distributed keyframe PGO matching Equation 6 (partial) |
| [`DISTRIBUTED_KEYFRAME_GRAPH_CHANGE_RECORD.md`](DISTRIBUTED_KEYFRAME_GRAPH_CHANGE_RECORD.md) | P1 — Direct factors and sparse remote keyframes for Equations 6 and 7 |
| [`EVALUATION_HARNESS_CHANGE_RECORD.md`](EVALUATION_HARNESS_CHANGE_RECORD.md) | P2 — Paper dataset configurations, and the evaluation harness |
| [`FIELD_COMMUNICATION_CHANGE_RECORD.md`](FIELD_COMMUNICATION_CHANGE_RECORD.md) | P2 — ROS + ZeroMQ field communication setup |

## Commits

| Commit | Subject |
|---|---|
| `680e941` | Unify intra- and inter-robot loop closure on the paper pipeline |
| `b3aa857` | Restore retained sources and add the session change record |
| `040b43a` | Bound inter-robot communications and move scans on demand |
| `30bdd8a` | Record the communications commit hash |
| `78edde3` | Propagate map-alignment uncertainty into cross-peer loop factors |
| `32052ff` | Record the map-alignment commit hash |
| `52ceb5f` | Restore the paper dataset configurations and check the contract |
| `ed6c4e8` | Add the paper evaluation harness |
| `90aa278` | Record the evaluation harness commit hash |
| `b71198b` | Add the ZeroMQ peer transport and inter-robot bridge |
| `e38dfd8` | Record the field communication commit hash |
| `b41a338` | Prepare pose graph for remote keyframe states |
| `d0ac2e7` | Implement direct distributed keyframe factors |

## Audit movement

| Item | Before | After |
|---|---|---|
| P1 — One pipeline for inter/intra loops | Missing | Implemented |
| P1 — Lightweight message pool | Unbounded, scan attached to every announcement | Implemented |
| P1 — Distributed keyframe PGO (Equation 6) | "Partial approximation", unanalysed | Analysed; multi-peer information matrices corrected; remote keyframe variables still absent |
| P2 — Paper dataset configurations | Absent | Implemented, with a parameter-contract test |
| P2 — Paper evaluation harness | Absent | Implemented and tested |
| P2 — ROS + ZeroMQ field communication | Absent | Implemented; bridge unbuilt |
| P2 — Pipeline test coverage | Registration and frames only | Ten suites |

At the close of the original 1 September session, one structural parity item
remained: **remote keyframe variables**, blocked on the symbol-space refactor
named in the audit. The 2 September continuation below records its
implementation and the fidelity work that replaces it as the open boundary.

## New components

| Component | Purpose | Tested here |
|---|---|---|
| `skid_loop_detection` | SOLiD distance, circular shift, implied yaw, candidate retrieval | Yes |
| `skid_solid_descriptor` | Descriptor construction, isolating SOLiD's global symbols | Yes |
| `skid_registration_params` | The one registration parameter set both nodes declare | Yes |
| `skid_comms` | Bounded queues, scan-cache policy, request tracking, transfer accounting | Yes |
| `skid_transport` | ZeroMQ peer transport and echo suppression | Yes, over real sockets |
| `evaluation/skid_eval` | The paper's metrics | Yes |
| `test/validate_config_parameters.py` | Parameter and launch contract checking | Yes |
| `liorf_zmqBridge` | ROS-to-ZeroMQ bridge | No — never compiled |
| `msg/ScanRequest`, `msg/ScanData` | On-demand scan channel | No — never generated |

## Defects found and fixed

Listed together because most were found by writing a test or attempting a
build, not by reading the code:

| Defect | Consequence | Found by |
|---|---|---|
| `SOLiD::ptcloud2bin` swaps its argument, so a retained bin aliased the node's reused scan buffer | Every queued announcement carried whatever the *next* scan overwrote it with | Wiring scan retention |
| A late `ScanData` reply never cleared a key's abandoned state | A peer that eventually answered stayed permanently blocked for that scan | Unit test |
| `getInitialGuesses` overwrote rather than accumulated each candidate's result | An accepted loop discarded when a later candidate in the same batch failed | Reading the refactor |
| Cross-peer loop factors composed through the map alignment with zero uncertainty | Overconfident factors with three or more robots | Reading `sendLoopThis` |
| `skid_pose_uncertainty` passed `nullptr` for a GTSAM `OptionalJacobian` | Does not compile against GTSAM 4.2 | Attempting the build |
| Coarse yaw from the A-SOLiD shift used one sector too many | A small constant error in every coarse guess | Deriving the shared helper |
| `publishContextInfoThread` looped with no wait | Spun a core whenever a peer was idle | Reading the loop |
| The self-addressed loopback announcement was broadcast to every peer | Would have defeated the descriptor-only saving entirely | Designing the split |
| An `or {}` coerced a mistyped manifest section | `expected: []` silently accepted | Unit test |
| A relative-only tolerance could never match an expected value of zero | An exact ATE match reported as a failure | Unit test |
| Paper-era `z_tollerance`/`rotation_tollerance` are ints where the node declares doubles | rclcpp refuses to construct the node | Parameter validator |

## Loose ends recorded nowhere else

- `.gitignore` gained `__pycache__/` and `*.pyc`, after a Python syntax check
  left compiled artifacts in the tree.
- `evaluation/skid_eval/__init__.py` is a package marker carrying the note that
  the harness is deliberately dependency-free.
- `include/Scancontext.{h,cpp}`, `nanoflann.hpp`,
  `KDTreeVectorOfVectorsAdaptor.h` and `tictoc.h` were removed in `680e941`
  and restored in `b3aa857` at the user's instruction. They remain in the tree
  and are built by nothing.

## Verification boundary for the whole session

This is the part worth reading before trusting any of the above.

**What was built and run here.** Ten test suites pass: seven C++ (registration,
loop detection, SOLiD descriptor, registration parameters, communications,
pose uncertainty, transport), the parameter-contract validator over 15
parameter files and 18 launch files, and the evaluation harness's 49 cases.
GTSAM, PCL, Eigen, ZeroMQ and the pinned solver submodules were installed
locally to make that possible; the transport suite was run 15 times
consecutively to rule out socket flakiness.

**What was not.** No ROS 2 node in this branch has been compiled or run.
`mapOptmization.cpp`, `mapFusion_so.cpp` and `zmqBridge.cpp` need `rclcpp`,
which this environment does not have, and the two new message types have never
been generated. Six slices of node-level change have now accumulated behind
that boundary. Every change record states its own gap, but the cumulative
position is:

1. A ROS 2 Lyrical `colcon build` of the whole branch has never happened.
2. No bag has been replayed, so no claim about intra-robot loop behaviour,
   on-demand scan transfer, cross-peer factor weighting, or dataset
   configuration is measured rather than implemented.
3. No two hosts have been bridged, and the bench procedure in
   `FIELD_COMMUNICATION.md` exists because none of it has been done.
4. Every threshold introduced this session is a reasoned default, not a
   calibration: registration gates, cache budgets, request timeouts,
   alignment floors, and the transport's high-water marks.

The tested components are the parts most likely to be subtly wrong — geometry,
policy, parsing, arithmetic — and the untested parts are mostly thin glue. That
is the intended shape, but it is not a substitute for a build.

### Post-session verification — 1 September 2026

A later follow-up closed part of the historical boundary above on ROS 2
Lyrical. The current `v3` working tree, including `LoopDiagnostic` and its bag
extractor, was built in the workspace with CMake/Make and package parallelism
capped at two. `liorf`, `benchmark_livox_bridge`, and `skid_slam_playback` all
completed; every ROS interface was generated, and the map-fusion and ZeroMQ
bridge executables linked.

All 12 `liorf` test targets pass. Eleven ran in the restricted test sandbox;
the ZeroMQ transport target was rerun with loopback socket access after its
only sandbox failure (`Operation not permitted`) and all 25 cases passed. The
Livox bridge's four conversion cases and the evaluation harness's 54 cases also
pass.

HelmDyn03 was then replayed in full at 1x, headless, with map fusion enabled and
a 180-second guard. The bridge forwarded all 3,226 LiDAR messages (32,228,928
points) and 32,261 IMU messages with no reported middleware losses or point
count mismatches; estimated odometry sampled at 19.98 Hz. The live ROS graph
contained `/jackal0/solid/loop_diagnostics` with type
`liorf/msg/LoopDiagnostic`. Because HelmDyn03 is a single-robot bag and the
initial robot does not perform inter-robot fusion, this validates construction,
message generation, topic wiring, and sustained replay—not candidate,
registration, PCM, scan-transfer, or ZeroMQ behaviour between robots.

The replay repeatedly emitted KISS-Matcher warnings that its ROBIN and solver
noise bounds were being clamped. Closures continued to be accepted, but those
warnings are field-calibration debt and should be resolved before treating the
run as an accuracy result.

### Post-session Equation (6) implementation — 2 September 2026

The symbol-space blocker was removed in `b41a338`: every locally owned pose
factor now uses an explicit GTSAM `X(index)` key, newest-pose lookups use the
known local key, and pose correction iterates the local keyframe count rather
than every value in iSAM2. This allows remote values to coexist without being
mistaken for local point-cloud indices.

Commit `d0ac2e7` then changed the default map-fusion route from algebraically
eliminated local-only loops to direct PCM-approved cross-robot factors. The
loop contract names both robot/keyframe endpoints and carries each endpoint's
pose in its owner's map. Every recipient keeps the Equation (7) subset of peer
poses in a separate symbol namespace and connects those observations with a
sparse owner-frame relative-motion tree. Registration factors retain their
full covariance; peer-motion edges have separately configured uncertainty
floors. Initial values derived from a registration are seeds, not duplicate
priors. Reversed endpoint reports are canonicalized so the same physical
measurement cannot be counted twice.

The full `liorf` package rebuilt on ROS 2 Lyrical, including both modified
nodes and the ZeroMQ bridge. All 13 non-socket CTest targets pass, and all 25
ZeroMQ transport cases pass with loopback access. The graph suite includes a
solver-level two-robot correction test plus key-space, sparse-trajectory,
orientation, and deduplication cases.

The parent workspace's new synthetic HelmDyn08/09 launcher is suitable for the
next integration check, but its documented derived
`HelmDyn08_09_two_robot_v1` bag is not present on the mounted dataset. It was
not regenerated implicitly because the provenance-preserving artifact is
approximately 1.1 GiB. Live two-pipeline factor delivery, RViz behaviour, and
trajectory accuracy therefore remain unclaimed.

The exact graph semantics, map/Earth separation, compatibility behaviour, and
remaining fidelity limits are in
[`DISTRIBUTED_KEYFRAME_GRAPH_CHANGE_RECORD.md`](DISTRIBUTED_KEYFRAME_GRAPH_CHANGE_RECORD.md).

## Suggested next steps

1. Generate or provide the provenance-preserving HelmDyn08/09 derived bag,
   replay both pipelines, and verify direct factor delivery and graph updates.
2. Bench-verify the ZeroMQ bridge with two bridges on one host.
3. Calibrate the KISS-Matcher noise bounds and registration gates on field
   data, along with the new remote-motion uncertainty floors.
4. Fill in each evaluation manifest's `expected` block once the protocol is
   confirmed against the paper.
5. Exchange revisioned peer corrections or original peer odometry/loop factors
   with covariance, and add factor retraction when a later PCM clique changes.
