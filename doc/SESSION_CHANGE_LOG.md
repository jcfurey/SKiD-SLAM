# Session Change Log

Session date: 1 September 2026

Branch: `claude/v3-paper-parity-wha23w`

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

One parity item remains: **remote keyframe variables** for full Equation (6)
parity, blocked on the symbol-space refactor named in the audit.

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

## Suggested next steps

1. **Build the branch on ROS 2 Lyrical.** This is the highest-value action
   available and gates everything else.
2. Replay a bag on one of the six restored dataset configurations, then a
   multi-robot bag.
3. Bench-verify the ZeroMQ bridge with two bridges on one host.
4. Fill in each evaluation manifest's `expected` block once the protocol is
   confirmed against the paper.
5. Only then attempt remote keyframe variables, which need the symbol-space
   refactor the audit describes and which fail silently rather than loudly if
   they are wrong.
