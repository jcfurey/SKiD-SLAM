# Change Record: Map-Alignment Uncertainty and the Equation (6) Analysis

Date: 1 September 2026

Branch: `claude/v3-paper-parity-wha23w`

Baseline: commit `30bdd8a`, *Record the communications commit hash*

Scope: the `v3` paper-parity item recorded in
[`PAPER_V3_GAP_AUDIT.md`](PAPER_V3_GAP_AUDIT.md) as *P1 — Distributed keyframe
PGO matching Equation 6*, and Phase 3 item 3.

______________________________________________________________________

## 1. Problem

The audit recorded this row as a "partial approximation" without saying what
the approximation actually was. Reading the code turned that into two specific
findings.

**The substantive one.** `gtsamExpressionGraph()` estimates the inter-map SE(3)
alignment `T_p` by weighted least squares and stores only the point estimate.
`sendLoopThis()` then uses it to bring a loop constraint's two endpoints back
into this robot's frame when they were registered against *different* peers:

```cpp
pose_to_this = { trans_this3.inverse() * pose_this.aligned_source_pose,
                 pose_this.covariance };   // covariance unchanged
```

The alignment is an estimate, and composing through it as if it were exact left
the resulting factor overconfident by exactly the alignment's error. The
mapping node consumes that covariance as a full `noiseModel::Gaussian`, so the
error goes straight into the pose graph's weighting.

This is reachable only with three or more robots. With two, both endpoints are
registered against the same peer, that peer's frame cancels algebraically, and
no alignment enters — so the two-robot case was, and remains, exact.

**A portability one, found while building.** `src/skid_pose_uncertainty.cpp`
passed `nullptr` for a GTSAM `OptionalJacobian` in two places. Against GTSAM
4.2 that is ambiguous between `OptionalJacobian`'s `Jacobian*`,
`Eigen::MatrixXd*` and `double*` constructors, and the file does not compile.

## 2. What changed

### 2.1 Uncertainty propagation

- `liorf::uncertainty::inverse()` — first-order propagation for the inverse of
  an uncertain pose, using the negated adjoint as the Jacobian.
- `gtsamExpressionGraph()` recovers the alignment's marginal covariance from
  its own optimization via `gtsam::Marginals`, adds a configured floor, and
  stores it in `_global_map_trans_covariance`.
- `mapAlignment()` returns the alignment pose, its covariance, and whether it
  is usable.
- `sendLoopThis()` composes through `inverse()` and `compose()` carrying that
  covariance, and drops the factor rather than emitting one whose propagated
  covariance is not valid.

### 2.2 Portability fix

Both ambiguous `nullptr` arguments became `{}`, a default-constructed
`OptionalJacobian`, which is the portable way to skip one.

### 2.3 Documentation

A new audit section, *Equation (6) and the two-level formulation*, records what
each level actually optimizes, how cross-robot information reaches a keyframe
graph, the one case where the two formulations are equivalent, the four ways
they are not, and the three code-level blockers to representing remote
keyframes.

## 3. Design decisions

### 3.1 A floor on the alignment uncertainty, not the bare marginal

The marginal from a well-conditioned fit over many registrations can be very
tight. It is also the wrong number on its own: it describes only the spread of
the fit, not drift between the two maps, and not the correlation described
below. The stored covariance is therefore `marginal + diag(floor²)`, and the
floor alone when the marginal cannot be recovered.

That keeps the system running when `Marginals` throws on an indeterminant
system — the alternative, dropping the factor, would silently stop multi-peer
loop closure on exactly the runs where it is hardest.

### 3.2 Independence is assumed, and it is the safe direction

The alignment was fitted to registrations including the one being composed with
it, so the two are correlated. Treating them as independent is an
approximation.

It is the right approximation to make here because its error has a known sign.
`compose()` adds `H C Hᵀ` terms, so a non-zero alignment covariance can only
enlarge the result — never shrink it. Carrying it therefore errs towards a
larger covariance and a weaker factor; treating the alignment as exact, as the
code did, erred towards a smaller covariance and a stronger one. A test pins
that ordering as a property of the composition chain rather than leaving it as
a claim in a comment.

### 3.3 Documenting the gap instead of closing it

The audit offered two branches: represent remote keyframe factors explicitly,
or document and validate an equivalent formulation. The second was taken, and
deliberately.

Remote keyframe variables require an explicit symbol space for local keyframes
first, because three sites in `mapOptmization.cpp` assume the value count
equals the keyframe count and that the highest key is the newest keyframe.
Those are exactly the assumptions that fail silently — a wrong pose written to
the wrong keyframe, or a marginal taken for the wrong variable — rather than
failing to compile. Making that change without the ability to build or replay
would be putting an unverifiable rewrite into the graph that everything else
depends on.

So the blockers are named precisely, in the order they have to be removed, and
the honest statement is that remote variables are not attempted here.

## 4. New parameters

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `mapfusion.interRobot.map_alignment_rotation_stddev_rad` | double | `0.05` | Rotational floor on the alignment uncertainty |
| `mapfusion.interRobot.map_alignment_translation_stddev_m` | double | `0.20` | Translational floor on the alignment uncertainty |

Both are validated at node construction and must be finite and positive.

## 5. Behavioural changes for operators

- **Three or more robots:** cross-peer loop factors are weaker than before, by
  the alignment's uncertainty. That is the correction — they were overweighted.
  Expect the fused map to move less abruptly on those factors.
- **Two robots:** nothing changes. No alignment enters that path.
- A cross-peer factor is now dropped, with a warning naming both robots, if the
  propagation produces an invalid covariance.
- Raising the floor defaults weakens multi-peer factors further; lowering them
  approaches the previous behaviour, which is not recommended.

## 6. Verification

### What was built and run

GTSAM 4.2 is available as a distribution package, so `skid_pose_uncertainty`
now builds and runs locally alongside the rest of the ROS-independent code.
That is new this session: this file's tests had not previously been executed
here.

| Suite | Result | Covers |
|---|---|---|
| `test_skid_pose_uncertainty` | pass (9 cases) | Existing covariance validation and PCM residual, plus: inverting twice restores pose and covariance exactly, the inverse stays positive definite, an invalid covariance is rejected, an uncertain left operand only adds uncertainty, the full cross-peer composition chain is conservative and leaves the measurement unchanged, and the diagonal floor is a valid covariance that can only grow when a marginal is added |
| The other six suites | pass | Unchanged; confirms no regression |

The `CarryingAlignmentUncertaintyIsConservative` test mirrors the exact
`inverse → compose → compose → between` chain `sendLoopThis()` performs, so the
composition order and the conservativeness property are both verified against
the real helpers rather than asserted.

The GTSAM 4.2 ambiguity in §1 was found by attempting the build, not by
reading.

### What was not verified

**The map-fusion node was not compiled or run**, for the usual reason: it needs
`rclcpp`. Specifically unverified:

1. That `gtsam::Marginals` on the single-variable `ExpressionFactorGraph`
   returns a usable covariance in practice, and how often it throws.
2. The magnitude of the correction — how much weaker cross-peer factors become
   on real data, which needs a three-robot bag.
3. That the floor defaults are the right order of magnitude. They currently
   mirror the PCM local-trajectory values, which is a starting point, not a
   calibration.
4. Whether the `nullptr` ambiguity also affects the GTSAM shipped with ROS 2
   Lyrical. The `{}` form is correct on both, so the fix is safe either way,
   but the original may or may not have compiled there.

## 7. Follow-up work

Still open:

- **P1** — Remote keyframe variables, blocked as described in §3.3 and the
  audit. The first step is an explicit symbol space for local keyframes.
- **P2** — Paper dataset configurations.
- **P2** — ZeroMQ field-communication transport or documentation.
- **P2** — The paper evaluation harness.

Newly opened:

- Calibrate the alignment floors against field data.
- Represent the correlation between the map alignment and the registrations it
  was fitted to, rather than assuming independence.
- Consider emitting a factor from a lone registration against a peer, which the
  current differencing scheme discards.

## 8. Commits

| Commit | Contents |
|---|---|
| this commit | Propagate map-alignment uncertainty into cross-peer loop factors |
