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
| [`PCM_COMMITMENT_CHANGE_RECORD.md`](PCM_COMMITMENT_CHANGE_RECORD.md) | P0/P1 — Correct Equation 11 direction and delay add-only graph publication |
| [`FULL_TWO_ROBOT_REPLAY_CHANGE_RECORD.md`](FULL_TWO_ROBOT_REPLAY_CHANGE_RECORD.md) | P1/P2 — Complete synthetic replay, graph-facing factor audit, and runtime bookkeeping |
| [`PCM_ABSOLUTE_GATE_CHANGE_RECORD.md`](PCM_ABSOLUTE_GATE_CHANGE_RECORD.md) | P0/P2 — Bound covariance-normalized PCM and formalize position-only scoring |
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
| `296171f` | Document distributed keyframe graph parity |
| `f610787` | Record synthetic two-robot bag derivation |
| `774594a` | Stabilize PCM publication and matcher diagnostics |
| `44e07df` | Add distributed factor audit tooling |
| `2a01982` | Fix map-fusion replay bookkeeping |
| `f70c7df` | Bound covariance-aware PCM cycle residuals |

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
| `skid_pcm_commitment` | Converts provisional PCM membership into monotonic publication authority | Yes |
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
| PCM consumed `source <- target` registrations where Equation (11) requires `target <- source` | Nonidentity consistency cycles did not represent the paper equation | Nonidentity two-robot cycle test |
| A background announcement used the latest cloud header instead of the queued keyframe's timestamp | Diagnostics associated valid factors with the wrong ground-truth poses | Two-robot factor audit |
| Every current maximum-clique member was published into an add-only graph immediately | A transient or undersized clique could become permanent graph state | Two-robot replay and graph-semantics review |
| Runtime PCM matrices were written beneath the installed package config | A symlink build modified a tracked source file during replay | Working-tree audit |
| Every keyframe entered absent and receive-only announcement queues | Communication reports claimed drops on links that did not exist | Complete two-robot replay |
| A late optimized fleet-map transform reused an older keyframe stamp | TF rejected a valid alignment update as old data | Complete two-robot replay |
| Replay teardown retained respawning children after their launch parent exited | Old validation processes survived until explicitly cleaned up | Host process audit |
| A highly uncertain false registration could buy several metres of Mahalanobis PCM tolerance | A 2.735 m endpoint-separation alias remained in the committed clique | Complete-run factor forensics and offline PCM reconstruction |
| HelmDyn mocap quaternions were treated as an SE(3) reference despite sign-invariant near-180-degree jumps | RTE/RRE mixed registration quality with an invalid orientation channel | Adjacent-orientation audit of both retimed trajectories |

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
eliminated local-only loops to direct cross-robot factors. Commit `774594a`
subsequently restricted that route to PCM-committed registrations. The
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

The parent workspace's synthetic HelmDyn08/09 fixture was subsequently
generated at `RESPLE_dataset/HelmDyn/HelmDyn08_09_two_robot_v1`. It contains
61,157 messages on four namespaced sensor topics over 141.768 seconds, both
retimed ground-truth trajectories, and source/output hashes in
`PROVENANCE.json`. The source bags were opened read-only and no partial output
was left behind. Live two-pipeline factor delivery, RViz behaviour, and
trajectory accuracy remain unclaimed until the derived bag is replayed.

The exact graph semantics, map/Earth separation, compatibility behaviour, and
remaining fidelity limits are in
[`DISTRIBUTED_KEYFRAME_GRAPH_CHANGE_RECORD.md`](DISTRIBUTED_KEYFRAME_GRAPH_CHANGE_RECORD.md).

### Post-session PCM stabilization and two-robot replay — 2 September 2026

Commit `774594a` closes the issues exposed by the first HelmDyn08/09 run. The
default KISS-Matcher noise gains now produce explicit 1.0 m ROBIN and 0.75 m
solver bounds, and validation prevents a clamping-enabled configuration from
silently exceeding the library's 1.0 m limit. Expected descriptor/registration
misses remain in the structured diagnostic stream but no longer flood the WARN
console. PCM now receives the correct `target <- source` measurement direction.

Because the receiving graph cannot retract a published factor, current
maximum-clique membership is provisional. A candidate must remain in a
supported clique for a configured number of consecutive recomputations before
it becomes a monotonic commitment. Only committed candidates affect map
alignment or either factor publisher. An evaluation switch can leave the whole
recognition/registration/PCM path active while disabling graph publication,
and diagnostics report provisional and committed state separately.

The same replay found that descriptor announcements leaving the background
FIFO inherited the newest cloud timestamp rather than their own keyframe time.
That did not change the stored keyframe index or registration geometry, but it
made earlier factor-to-ground-truth scoring invalid. `ContextInfo` now carries
the descriptor's captured timestamp. PCM scratch matrices also default to a
runtime temporary directory instead of the installed/source config directory.

After rebuilding, a corrected factor-enabled 70-second segment ran both
pipelines at 2x. Each bridge forwarded 1,400 LiDAR and 14,001 IMU messages with
no reported loss or mismatch. The trace retained 8,566 descriptor candidates,
86 accepted registrations, and 23 commitment transitions. Thirty-seven unique
direct factors were delivered symmetrically to both endpoint topics. For the
30 factors whose two timestamps were each within 30 ms of retimed mocap, the
translation RTE median was 0.16 m, p90 was 0.40 m, and maximum was 1.33 m. The
other seven lie in mocap gaps and are excluded from that strict result.

The current ROS 2 Lyrical build passes all 14 non-socket CTest targets. The
transport target passes all 25 cases with loopback socket access. Detailed
parameter semantics, evidence, and limitations are in
[`PCM_COMMITMENT_CHANGE_RECORD.md`](PCM_COMMITMENT_CHANGE_RECORD.md).

### Post-session complete two-robot audit — 2 September 2026

The full 141.768-second HelmDyn08/09 timeline was replayed on isolated domains
193 and 194. Domain 193 delivered every one of the fixture's 61,157 sensor
messages through the two bridges without reported loss or mismatch. Domain 194
preserved a 43.8 MiB diagnostic MCAP containing 53,964 diagnostics, 209
accepted registrations, and 111 factors on each endpoint topic.

Commit `44e07df` adds a repeatable graph-facing audit. It verifies factor
identity, orientation, measurement equality, recipient coverage, duplicates,
and endpoint timestamp consistency before scoring RTE/RRE against original
keyframe times. The full artifact has exact two-sided delivery. Ninety-six of
111 factors associate within 30 ms of retimed mocap; RTE is 0.145 m median and
0.306 m p90, but one clear separation outlier and the unresolved orientation
convention prevent calling this a final accuracy result.

The long run also exposed fictitious drop counts from unused announcement
queues and an old timestamp on a newly optimized fleet-map transform. Commit
`2a01982` gates both sides of each announcement route with the same one-producer
policy, stamps alignment state when published, and makes respawn configurable
for deterministic replay shutdown. A post-fix complete-timeline smoke test on
domain 195 produced zero queue drops and no stale-TF, oversized-noise, ERROR,
or FATAL log entry. See
[`FULL_TWO_ROBOT_REPLAY_CHANGE_RECORD.md`](FULL_TWO_ROBOT_REPLAY_CHANGE_RECORD.md).

### Post-session covariance-bounded PCM audit — 2 September 2026

The complete factor trace resolved the earlier HelmDyn ambiguity. Its readme
provides only a translation correction, while the two quaternion streams have
20 and 43 sign-invariant adjacent jumps above 90 degrees. They are now treated
as position-only ground truth. The factor auditor has an explicit mode that
suppresses invalid RTE/RRE, reports endpoint-separation error, and can enforce
that quantity as a regression gate.

The `jackal0/164 <-> jackal1/59` failure was a real scan alias. Its estimated
endpoint separation was 3.495 m against a 0.760 m mocap separation, and its
2.9 m cycle inconsistency was normalized by translation uncertainty as large
as 8.82 m. Honest factor uncertainty was therefore weakening outlier
selection. PCM retains its covariance-normalized Equation (11) test and now
optionally applies independent absolute translation and rotation ceilings to
the same residual. Both are disabled by default; the HelmDyn profile enables
a 1.0 m translation ceiling.

An offline replay of the original 111-factor graph yields a 108-factor maximum
clique at that ceiling and excludes the alias. A fresh complete 2x replay on
isolated ROS domain 200 recorded 89 factors on each endpoint topic with exact
delivery symmetry. Seventy-seven associate to mocap positions within 30 ms:
separation error is 0.054 m median, 0.167 m p90, and 0.214 m maximum. Of 165
associated registrations before PCM, 76 were wrong by more than 0.25 m and
none of those reached a graph. Full evidence and limitations are in
[`PCM_ABSOLUTE_GATE_CHANGE_RECORD.md`](PCM_ABSOLUTE_GATE_CHANGE_RECORD.md).

## Suggested next steps

1. Bench-verify the ZeroMQ bridge with two bridges on one host, then over the
   target radio.
2. Calibrate registration, absolute PCM ceilings, commitment policy, and
   remote-motion uncertainty on real simultaneous multi-robot field data.
3. Fill in each evaluation manifest's `expected` block once the protocol is
   confirmed against the paper.
4. Exchange revisioned peer corrections or original peer odometry/loop factors
   with covariance, and add factor retraction when a later PCM clique changes.
