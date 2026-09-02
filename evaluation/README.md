# Evaluation harness

Reproducible measurement of the metrics the paper reports: place-recognition
precision and recall, registration RTE/RRE and success rate, trajectory ATE and
ARE, descriptor memory, and communication cost.

The harness has no dependency beyond PyYAML, which this package already needs
for its parameter files. An evaluation that requires a scientific stack to be
installed is one that does not get run.

## Running

```
./evaluation/run_evaluation.py evaluation/manifests/park.yaml \
    --results-root results/park
```

Add `--json` for machine-readable output, `--output FILE` to write to a file.

Every section is optional. A manifest that names no candidate file reports no
place-recognition figures, and says why, rather than failing: a missing input
is a gap in the evaluation, not an error in it.

## File conventions

### Trajectories: TUM format

```
# timestamp tx ty tz qx qy qz qw
1614556800.123456789 1.234 5.678 0.910 0.0 0.0 0.0 1.0
```

Timestamps in seconds, translation in metres, quaternion in `(x, y, z, w)`
order. Comments start with `#`. Ground truth and estimates use the same
convention so the two can never be accidentally mismatched. A malformed line
is an error naming the line number, not a silent skip, because a quietly
dropped row becomes a quietly wrong metric.

### Loop candidates: CSV

```
query_time,match_time,score[,is_true_loop]
```

`score` is the descriptor distance, so smaller is a better match. Leave
`is_true_loop` out and the harness labels each candidate from ground truth: a
true loop is one whose two ground-truth positions are within
`revisit_distance_m` and whose observations are at least `min_time_gap_s`
apart. Without that time gap every consecutive pose is trivially a "loop".

### Registrations: CSV

```
query_time,match_time,tx,ty,tz,qx,qy,qz,qw
```

The transform maps the match frame into the query frame, matching the relative
pose the pose graph consumes. Ground truth for each is derived from the
reference trajectory at those two timestamps, so only the estimate is supplied.

### Communication: the map-fusion log

Point `comms_log` at the map-fusion node's stdout. The harness reads the
periodic diagnostics line `MapFusion::commsMaintenance` emits, whose counters
are cumulative, so the last one in the log is the run total.

## Producing the inputs

`extract_from_bag.py` converts an odometry topic in a ROS 2 bag into a TUM
trajectory. It has not yet been validated against a representative recorded
evaluation run, so check its topic and frame convention before trusting a
metric computed from it.

Map fusion publishes the other two inputs as a structured
`liorf/msg/LoopDiagnostic` trace. The default per-observer topic is
`/<robot>/solid/loop_diagnostics`; record the topic from the robot doing the
fusion, then extract both CSVs in one pass:

```bash
./evaluation/extract_diagnostics_from_bag.py my_recorded_run \
    --topic /jackal1/solid/loop_diagnostics \
    --candidates-output results/park/candidates.csv \
    --registrations-output results/park/registrations.csv
```

The candidate CSV includes every eligible descriptor comparison in the KNN
set, including threshold and candidate-budget rejections. Position-search
fallbacks are not included because they have no descriptor score. The
registration CSV includes accepted registrations only and writes the estimated
transform in the documented match-into-query convention. Extra identity,
quality, uncertainty, and timing columns are retained for auditability; the
harness ignores columns it does not consume.

The same topic also records scan-request outcomes and PCM maximum-clique
decisions. Those stages are intentionally excluded from the two CSVs, but stay
available in the bag for diagnosing why a descriptor candidate did not become
a factor.

## Expected values

Each manifest has an `expected` block. It is deliberately **not** filled in.

The figures in the paper are reported over the authors' runs, and their exact
evaluation protocol — association window, alignment, revisit radius, success
thresholds — is not reproduced here. Copying the numbers in would assert a
comparison that has not been established. Fill them in once the protocol above
is confirmed to match, and the runner will check each measurement against its
value, within a per-metric relative and absolute tolerance.

## What the metrics mean

| Metric | Definition |
|---|---|
| ATE | RMSE of translation error after aligning the estimate onto the reference. Measures global consistency. |
| ARE | RMSE of the geodesic rotation angle after the same alignment, in degrees. |
| RPE | Error of relative motion over a fixed index step. Alignment-free by construction, so it measures drift rather than global consistency. |
| RTE / RRE | Translation and rotation error of one registered loop against its ground-truth relative transform. |
| Success rate | Fraction of registrations inside **both** thresholds. A loop with the right position and the wrong heading is not a success. |
| Max F1 | Best F1 over all acceptance thresholds, with the threshold that achieves it. |
| Average precision | Precision weighted by the recall it gains; the area under the precision-recall curve. |
| Recall at full precision | The most recall available while admitting no false positive at all. |

Alignment is part of the ATE definition, not preprocessing: an absolute error
measured in the wrong frame measures the frame. Use `rigid` when the estimate
is metric, and `sim3` only when its scale is genuinely unobserved — a
similarity alignment will hide a real scale error. `yaw` restricts the
alignment to a rotation about z for planar platforms.
