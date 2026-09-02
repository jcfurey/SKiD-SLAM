# Covariance-Bounded PCM and HelmDyn Ground-Truth Audit

Date: 2 September 2026

Branch: `v3`

Implementation commit: `f70c7df` (*Bound covariance-aware PCM cycle
residuals*)

Paper target: Equation (11) pairwise consistency and registration evaluation
in `2505.08230v3`

## Why this follow-up exists

The first complete HelmDyn08/09 replay delivered 111 factors symmetrically,
but one committed registration, `jackal0/164 <-> jackal1/59`, was a clear
geometric alias. Its estimated endpoint separation was 3.495 m while the
retimed mocap positions were 0.760 m apart. The resulting 2.735 m separation
error was too large to dismiss as a frame convention.

The same audit reported large RRE values and a second factor with large vector
RTE but nearly correct endpoint separation. Before changing PCM, those two
effects had to be separated: one was a real registration error, while the
other could be an invalid use of the dataset's orientation channel.

## HelmDyn ground truth is position-only here

The HelmDyn dataset readme supplies only a LiDAR translation correction for
sequences 05--10:

```text
t_L_gt: [-0.03, -0.03, -0.0185]
```

It does not publish a body-to-LiDAR rotation. More importantly, the raw mocap
quaternions are not a continuous platform attitude reference. Sign-invariant
adjacent rotation differences contain 20 jumps above 90 degrees in the
retimed HelmDyn08 trajectory and 43 in HelmDyn09; the respective maxima are
179.13 and 179.80 degrees. These cannot be explained by a fixed extrinsic or
by the harmless `q` versus `-q` quaternion representation.

Consequently, HelmDyn08/09 is treated as position-only ground truth for this
factor audit. RTE and RRE derived from the supplied orientations may still be
printed as forensic diagnostics, but they are not accuracy claims. The
defensible registration quantity is endpoint-separation error:

```text
abs(norm(estimated translation) - distance(mocap endpoint positions))
```

That scalar is invariant to global translation, endpoint direction, and the
unknown rotation convention. The small published LiDAR translation correction
also cancels between the two platforms because both trajectories share the
same mocap frame.

`score_loop_factors_from_bag.py --position-only-ground-truth` now makes this
contract explicit. It writes `null` RTE/RRE values instead of silently using
untrusted orientations, identifies the report as `position_only`, and supports
`--max-separation` as an executable regression gate.

## Root cause of the committed alias

The rejected baseline measurement was not caused by timestamp association or
a local odometry jump:

- query `jackal0/164` at `1724418963.4722502`;
- match `jackal1/59` at `1724418952.3822877`;
- descriptor distance 0.00415;
- 19 coarse correspondences, 6 translation inliers, and 1,799 fine inliers;
- overlap 0.986 and truncated MSE 1.19433 m^2;
- estimated translation `[-2.5123, 1.0993, 2.1670]` m;
- neighboring `jackal0/162 -> jackal1/59` and
  `jackal0/163 -> jackal1/59` registrations had only 0.018 m and 0.033 m
  separation error; and
- local jackal0 motion was 0.574 m from keyframe 162 to 163 and 0.511 m from
  163 to 164.

The registration covariance correctly described the bad measurement as very
uncertain: its translation principal standard deviations were approximately
1.90, 2.29, and 8.82 m, and its rotational principal standard deviations were
18.7, 27.6, and 31.7 degrees. That is appropriate for factor weighting, but it
made the same 2.9 m PCM cycle residual inexpensive in Mahalanobis distance.
The candidate remained in the maximum clique through 143 recomputations and
was therefore committed by the delayed-publication policy.

This exposes a separation of responsibilities. Registration covariance should
remain honest and conservative when weighting a graph factor. It must not buy
unbounded geometric tolerance in the outlier-selection layer.

## Implemented gate

Equation (11)'s covariance-normalized gate remains mandatory. PCM now also
supports independent absolute ceilings on the translation and rotation of the
same SE(3) cycle residual:

| Parameter | Default | HelmDyn08/09 | Meaning |
|---|---:|---:|---|
| `mapfusion.interRobot.pcm_threshold` | 4.0 | 1.0 | Strict upper bound on Mahalanobis distance |
| `mapfusion.interRobot.pcm_max_translation_residual_m` | 0.0 | 1.0 m | Absolute cycle-translation ceiling; zero disables it |
| `mapfusion.interRobot.pcm_max_rotation_residual_rad` | 0.0 | 0.0 | Absolute cycle-rotation ceiling; zero disables it |

The package default preserves the paper equation without imposing a
dataset-specific scale. The HelmDyn playback profile opts into the 1.0 m
translation ceiling. Node construction rejects non-finite or negative values,
and startup logs expose all three active thresholds.

The policy is isolated in `pcmResidualPassesGate()`. Tests prove that an
inflated covariance cannot make a 3 m residual pass a 1 m absolute ceiling,
that translation and rotation ceilings are independent, that zero disables an
axis, and that invalid thresholds fail closed.

## Deterministic replay of the original PCM graph

The 111 originally committed factors were reconstructed offline with their
recorded poses and covariances. Adding only an absolute translation-cycle
ceiling produced these maximum-clique sizes:

| Ceiling | Maximum clique |
|---:|---:|
| 0.5 m | 106 |
| 1.0 m | 108 |
| 1.5 m | 108 |
| 2.0 m | 108 |
| 2.5 m | 109 |
| 3.0 m | 109 |

At 1.0 m the known `164 <-> 59` alias has degree one and is outside the
108-factor clique. The other excluded measurements are `jackal0/235 <->
jackal1/117` and `jackal0/457 <-> jackal1/258`. The known alias has no cycle
neighbors below 0.5 m and only one below 1.0 m, while its unweighted cycle
translation residual is about 2.9 m against each nearby good registration.

## Full live replay

The changed package was rebuilt and the complete derived HelmDyn08/09 bag was
replayed headlessly at 2x on isolated ROS domain 200. Evidence is retained at:

```text
build/skid_validation/helmdyn08_09_pcmcap_20260902_domain200/
```

The sequentially readable 109.6 MiB MCAP contains 116,198 messages, including
43,662 PCM diagnostics, 19,169 descriptor-candidate rows, 185 accepted
registrations, and 89 `LoopConstraint` messages on each endpoint topic. The
factor auditor found exact two-sided delivery, no duplicate or mismatched
measurement, 241 consistent endpoint timestamps, and no key-index time
regression.

Seventy-seven of the 89 factors have both endpoints within 30 ms of a retimed
mocap sample. Their position-only separation errors are:

| Metric | Value |
|---|---:|
| median | 0.054 m |
| p90 | 0.167 m |
| maximum | 0.214 m |

For comparison, the unbounded complete run had 0.050 m median, 0.153 m p90,
and 2.735 m maximum separation error. The new run's raw full-pose diagnostic
is 0.139 m median, 0.307 m p90, and 2.066 m maximum RTE, but this is not a
calibrated HelmDyn result because it uses the invalid orientation channel.

The selection pressure is visible before graph publication. Of 165 accepted
registrations that associate within 30 ms, 76 exceed 0.25 m separation error,
60 exceed 1 m, and 45 exceed 2 m. None of those 76 measurements became a
factor; every associated committed factor is within 0.25 m.

Keyframe sampling is scheduling-dependent, so the live run did not recreate
the exact baseline keyframe pair. Its nearest equivalent query used timestamp
`1724418963.5221412` and the same match timestamp, but was candidate rank 13
and stopped at the five-candidate budget before registration. The deterministic
recorded-graph replay above is therefore the direct test of the original
alias, while domain 200 is the end-to-end test of the deployed profile.

## Verification

`liorf` and `skid_slam_playback` rebuilt from source on ROS 2 Lyrical. All 14
non-socket CTest targets pass, including the PCM uncertainty, configuration-
contract, evaluation, and launch-merge suites. The factor-audit CLI passes the
domain 200 artifact with 77 associated factors and a 0.25 m maximum-separation
gate, then deliberately fails the same artifact with a 0.20 m gate on its
measured 0.214054 m maximum.

## Remaining boundary

This is still a synthetic pairing of separately recorded trajectories in one
motion-capture arena. The 1.0 m ceiling is justified for that bounded arena,
not established as a universal field default. A real simultaneous multi-robot
run should calibrate translation and rotation ceilings against vehicle speed,
keyframe spacing, local-odometry uncertainty, and environment scale.

PCM commitment is still monotonic because the receiving iSAM2 graph cannot
remove a published factor. Absolute residual ceilings prevent covariance from
making arbitrarily bad cycles compatible, but they do not replace revisioned
peer state or factor retraction when later evidence invalidates an already
published constraint.
