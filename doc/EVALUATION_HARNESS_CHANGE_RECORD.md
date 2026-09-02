# Change Record: Dataset Configurations and the Evaluation Harness

Date: 1 September 2026

Branch: `claude/v3-paper-parity-wha23w`

Baseline: commit `32052ff`, *Record the map-alignment commit hash*

Scope: the two remaining `v3` P2 items other than ZeroMQ — *Paper dataset
configurations* and *Paper evaluation harness* — and Phase 4 items 3 and 4.

______________________________________________________________________

## 1. What was missing

The package could not be pointed at any of the paper's field datasets, and
nothing measured whether a run reproduced the paper's numbers. The audit
listed both as P2, and both had the same underlying problem: the paper-era ROS
1 tree had the configurations, and nothing had the metrics.

## 2. Dataset configurations

Six configurations — GEODE, GRACO, Majang rover, Moon, Park, STEAM legged —
ported from commit `0611f71` to the ROS 2 schema, each with a launch file.

Each file's header records what the port dropped, what it added, and where a
value had to change, so package defaults are never mistaken for dataset
measurements. Dropped: the LOAM feature-extraction thresholds and
`mappingCornerLeafSize`, which belonged to the corner/surface split liorf
removed; `odometrySurfLeafSize`; `timeField`, now derived from the sensor
type; and the `navsat`/`ekf_gps` blocks, which no launch file in this package
starts. Added with no paper-era counterpart: `imuRate`, `point_filter_num`,
`surroundingKeyframeMapLeafSize`, `loopClosureICPSurfLeafSize`, the geographic
frame settings, and the `loopClosure` block.

Two deviations are worth naming:

- **GEODE** specified `surroundingKeyframeDensity: 0`. That value is a PCL
  voxel leaf size, and zero is not a usable one. It is the package default
  here, and the file says so.
- **Moon** pairs an Ouster point-cloud topic with `sensor: velodyne`. That is
  what the original said, and it is consistent with a bag whose ring and time
  fields were converted, so it is preserved rather than "fixed", with a note to
  check it against the data.

### The parameter-contract validator

`test/validate_config_parameters.py` checks every parameter file against the
parameters the sources actually declare, and every launch file against the
assets it names.

This exists because of how ROS 2 fails here. A key nobody declares is ignored
silently and simply has no effect; a key declared with a different type stops
the node from starting. Neither shows up in review.

The validator was deliberately written and run against the eight existing
configurations *before* the six new ones, so it had a trusted baseline. It then
earned its place immediately: the paper-era `z_tollerance` and
`rotation_tollerance` are integers where the node declares doubles, which
rclcpp rejects at construction. A naive port would have produced six
configurations that cannot start a node.

It resolves the `mapfusion.registration.*` and `liorf.loopClosure.registration.*`
names that `declareConfig` builds at runtime from a prefix and a suffix; a
literal grep would report all 29 of each as undeclared.

## 3. Evaluation harness

`evaluation/` implements the metrics and the file conventions around them.

| Module | Contents |
|---|---|
| `linalg` | 3x3/4x4 dense algebra, Jacobi eigen decomposition, quaternion conversions |
| `trajectory` | TUM parsing and writing, one-to-one time association |
| `alignment` | Horn closed-form rigid, Sim(3), and yaw-only alignment |
| `metrics` | ATE, ARE, RPE, registration RTE/RRE, success rate, summary statistics |
| `place_recognition` | Ground-truth labelling, precision-recall curve, max F1, average precision |
| `resources` | Descriptor memory against a Scan Context baseline; parsing the map-fusion diagnostics line |
| `manifest` | Dataset manifest schema and validation |
| `runner`, `report` | Orchestration and rendering |

`run_evaluation.py` is the CLI; `evaluation/manifests/` holds one manifest per
dataset.

## 4. Design decisions

### 4.1 No numeric dependency

The harness uses only PyYAML, which the package already needs. Horn's method
requires the dominant eigenvector of a 4x4 symmetric matrix, which a local
Jacobi routine supplies in about fifty lines.

The alternative was NumPy. For a ROS package that is a new rosdep on a test
path, and an evaluation that will not run unless a scientific stack is
installed is one that does not get run. The local routine is tested against
cases with known eigenvalues and against exact recovery of a known transform.

### 4.2 Alignment is part of the ATE definition

`absolute_trajectory_error` aligns before measuring, because an absolute error
computed in the wrong frame measures the frame rather than the trajectory. The
manifest chooses the alignment, and the README says plainly that `sim3` will
hide a real scale error and should only be used when scale is genuinely
unobserved. A test pins exactly that: a 10% scale error survives rigid
alignment and vanishes under Sim(3), while RPE reports it either way.

### 4.3 A missing input is a gap, not an error

A manifest naming no candidate file produces no place-recognition figures and
says why. Failing the whole run instead would mean a partial evaluation — the
normal case while instrumentation is still being added — produces nothing at
all.

Malformed *content* is the opposite: a TUM line with the wrong field count is
an error naming the line number, not a silent skip, because a quietly dropped
row becomes a quietly wrong metric.

### 4.4 The `expected` blocks are empty on purpose

Every manifest has an `expected` block, and every value in it is null.

The paper's figures are reported over the authors' runs, and their exact
protocol — association window, alignment, revisit radius, success thresholds —
is not reproduced here. Filling the numbers in would assert a comparison that
has not been established, and the harness would then report "within tolerance"
against a protocol difference rather than a real agreement. The block is
documented as a placeholder to fill once the protocol is confirmed.

### 4.5 Success needs both thresholds

A registration counts only when translation and rotation are both acceptable. A
loop with the right position and the wrong heading is not a successful
registration, and a test asserts that a 45-degree error at zero translation
scores zero.

## 5. What the node could not supply at this change

At the time of this change, two harness inputs could not be extracted from a
bag; that was the useful finding from building it:

1. **Loop-candidate scores.** Precision-recall needs the descriptor distance of
   every retrieved candidate, including rejected ones. The nodes log that at
   debug level and publish it nowhere. The audit already tracks the missing
   structured diagnostic topic as Phase 2 item 4; this is the concrete
   consumer that needs it.
2. **Registrations.** `LoopConstraint` carries keyframe indices, and the
   harness keys on timestamps. The `cloud_info` topic carries both, so a bag
   containing it is sufficient, but no extractor is written.

Both are recorded in `evaluation/README.md` and in the audit row rather than
left for someone to discover.

### Follow-up — 1 September 2026

Both gaps are now closed. `liorf/msg/LoopDiagnostic` publishes eligible
descriptor scores (including rejections), keyframe identities/timestamps, scan
outcomes, full registration results, and PCM decisions.
`evaluation/extract_diagnostics_from_bag.py` converts that topic into candidate
and accepted-registration CSVs in one pass. Its filtering and transform
convention have unit coverage and were exercised through an actual ROS 2 bag
serialization round trip. A representative multi-robot dataset run is still
required before any paper comparison can be claimed.

## 6. Verification

### What was built and run

| Suite | Result | Covers |
|---|---|---|
| `test_skid_eval` | pass (49 cases) | Quaternion round trips, rotation angle including the acos clamp, Jacobi eigen against known eigenpairs; TUM parsing and its four rejection cases, one-to-one association, the association time window; exact recovery of a known rigid transform, Sim(3) scale recovery, yaw-only alignment, degenerate inputs; ATE vanishing under a pure frame change, scale surviving rigid alignment and vanishing under Sim(3), RPE measuring per-step drift and being frame-invariant, registration errors and both-threshold success; perfect and interleaved PR curves, tie handling, ground-truth labelling, recall over proposals versus over truth; descriptor sizes and the 12x Scan Context ratio, parsing the real diagnostics line; manifest defaults and five rejection cases; and an end-to-end run over a synthetic circular trajectory with known answers |
| `validate_config_parameters` | pass | 14 parameter files, 17 launch files |
| The seven C++ suites | pass | Unchanged |

The CLI was run against a synthetic dataset and produced a correct report:
ATE and ARE zero for an estimate differing only by a frame, max F1 of 1.0 where
the two true revisits score below the two non-revisits, and a success rate of
0.5 where one of two registrations is 5 m out against a 2 m threshold.

Two real defects were found by these tests rather than by inspection:

- `expected: []` in a manifest was silently coerced to an empty mapping by an
  `or {}`, so a mistyped section passed validation. Every optional section now
  rejects a non-mapping instead.
- The expected-value comparison used a purely relative tolerance, which is
  vacuous when the expected value is zero — an ATE of exactly 0.0 could not be
  matched. Each metric now carries a relative and an absolute tolerance.

### What was not verified at this change

- **No dataset has been run.** None of the six configurations has been used
  against its bag. Topic names, `imuRate` — a package default, not a
  measurement — and the Moon sensor/topic pairing are all unchecked against
  real data.
- **No metric has been computed from a real run.** Every number above comes
  from synthetic data with a known answer. That validates the implementations,
  not their agreement with the paper.
- **`extract_from_bag.py` had not been executed.** The original environment had
  no ROS installation. The later ROS follow-up has still not validated this
  trajectory extractor against a representative recorded estimate.
- The `python3-pytest` and `python3-yaml` test dependencies were added to
  `package.xml` but not resolved through rosdep here.

## 7. Follow-up work

Still open:

- **P1** — Remote keyframe variables for full Equation (6) parity.
- **P2** — ZeroMQ field-communication transport or documentation. This is now
  the only untouched row in the audit.

Newly opened:

- Publish a structured diagnostic topic carrying per-candidate descriptor
  distances, so place-recognition recall can be measured at all.
- Write the keyframe-index-to-time extractor for registrations.
- Run each dataset and fill in the `expected` blocks once the protocol is
  confirmed against the paper.

The first two items were completed by the follow-up above. Dataset execution
and protocol confirmation remain open.

## 8. Commits

| Commit | Contents |
|---|---|
| `52ceb5f` | Restore the paper dataset configurations and check the contract |
| `ed6c4e8` | Add the paper evaluation harness |
