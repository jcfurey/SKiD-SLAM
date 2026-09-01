# Geographic and Multi-Platform Frame Architecture

Status: design audit and implementation proposal

Audit date: 2026-09-01

Audited SKiD-SLAM revision: `99368de85ff293a92ef38fdd96835a116a191194`

## Decision

SKiD-SLAM's current frame handling is sufficient for a single local replay in
which `map` and `odom` are effectively aliases. It is not sufficient for a
fleet whose platforms may operate at geographically separated sites.

There must be one canonical `earth` frame in any combined TF graph. Each
platform, or each local operating site, may have its own `map`, `odom`, and
`base_link` chain. UTM may be exposed as a zone-scoped projected coordinate
system, but it must not be used to reconcile multiple independent `earth`
frames.

The required fleet topology is:

```text
earth                         WGS-84 ECEF; one fleet-wide authority
├── platform_0/map            local ENU frame at datum 0
│   └── platform_0/odom       continuous local odometry
│       └── platform_0/base_link
└── platform_1/map            local ENU frame at datum 1
    └── platform_1/odom       continuous local odometry
        └── platform_1/base_link
```

Robots at the same site may share a site map instead:

```text
earth
└── site_alpha/map
    ├── platform_0/odom
    │   └── platform_0/base_link
    └── platform_1/odom
        └── platform_1/base_link
```

The site-map form is valid only after both robots are demonstrably expressed
in that same map. Until GNSS, surveyed control, or an accepted inter-robot
registration establishes the relationship, the map trees must remain
detached. Publishing an identity transform is not a substitute for an unknown
transform.

## Can every platform have its own `earth` frame?

Not in one integrated TF tree. REP-105 defines `earth` as the Earth-centered,
Earth-fixed (ECEF) origin specifically so that multiple local maps can share a
common reference. Multiple frames named or treated as separate earth origins
would produce disconnected roots and leave the fleet transform undefined.

A robot running in an isolated ROS domain can internally use the conventional
name `earth`. When its data is bridged into the fleet graph, however, frame IDs
must be rewritten into one canonical fleet tree and all global positions must
refer to the same WGS-84 ECEF frame. It is simpler and safer to make this
contract explicit from the outset.

The frame that should be platform-specific is `map`, not `earth`.

## Where UTM fits

UTM is useful, but it solves a different problem. It projects geodetic
latitude/longitude into zone, hemisphere, easting, and northing coordinates.
It does not create the missing relationship between independent earth frames.

### One UTM zone

If every platform operates in one UTM zone and over a mission-scale area, a
shared, explicitly named projection such as `utm_16N` can be useful to
operators and planning software:

```text
earth
└── site_alpha/map            exact local ENU/ECEF anchor
    └── platform_0/odom
        └── platform_0/base_link

geodesy service/output:
  site_alpha/map pose <-> UTM zone 16N easting/northing
```

Some systems instead expose `utm_16N` as a TF parent of local world frames.
That can be practical within one zone, provided the zone and hemisphere are
part of the frame identity and the deployment accepts projected-grid
distortion. It is not the preferred core representation for SKiD because its
local map uses float point coordinates.

### Multiple UTM zones

UTM eastings and northings are not globally unique. The tuple must retain at
least zone and hemisphere, and a robot crossing a zone boundary changes
projection. Easting/northing values from different zones must never be
subtracted or connected by an assumed identity transform.

The correct cross-zone path is:

```text
UTM(zone A) -> geodetic/WGS-84 -> ECEF earth -> geodetic/WGS-84 -> UTM(zone B)
```

This conversion is a map projection, not one globally valid rigid-body
transform. TF can represent the rigid pose of a local ENU map in ECEF, but it
cannot itself implement the nonlinear UTM projection over an entire zone.
Accordingly, UTM conversion should live in a geodesy component or
zone-qualified output interface. The current `robot_localization`
implementation makes the same distinction: its ECEF `earth` transform is
supported with local Cartesian coordinates, not with UTM mode.

### Numeric reason to keep SLAM local

SKiD stores point coordinates in `pcl::PointXYZI` and performs much of its map
geometry with `Eigen::Affine3f`. ECEF translations are about 6.4 million
metres, where a 32-bit float has roughly half-metre spacing. UTM northings can
also be several million metres and lose centimetre-level resolution in float.

Therefore:

- keep scans, keyframes, ICP, and local maps near a local origin;
- keep `earth -> map` and geodetic conversion in double precision;
- never translate the PCL map itself into raw ECEF or large UTM coordinates;
- convert only poses and explicitly requested products at the global boundary.

## Current SKiD-SLAM behavior

### Published topology

The default parameters define only `map`, `odom`, and body frames in
[`utility.h`](../include/utility.h#L183-L186). No `earthFrame` parameter or
`earth` transform publisher exists.

Each transform-fusion instance initializes its six-component fusion transform
to zero and publishes:

```text
map -> <robot_id>/odom
```

The relevant implementation is in
[`imuPreintegration.cpp`](../src/imuPreintegration.cpp#L79-L125). Because
`mapFrame` defaults to the shared, unprefixed string `map`, two robots appear
co-located until map fusion supplies a relative transform. An unknown
relationship is therefore represented as identity.

The TestTrack launch is safe from cross-robot TF collisions only because it
runs one robot in an isolated ROS domain and disables map fusion. In that
configuration, `map` and `jackal0/odom` are effectively coincident.

### GPS conversion

The GPS handler in
[`mapOptmization.cpp`](../src/mapOptmization.cpp#L389-L411):

1. resets `GeographicLib::LocalCartesian` from the first accepted fix;
2. converts later fixes relative to that private origin;
3. discards the datum from the published interface; and
4. hardcodes the result's frame ID to `map`.

Consequently, two distant robots both turn their first fix into local
`(0, 0, 0)` coordinates and label the unrelated results with the same frame
name. The code also ignores a configured non-default `mapFrame` in this path.

In addition, the GPS subscription uses `gpsTopic` directly rather than the
robot-prefixed topic helper used for LiDAR and IMU. With the common multi-robot
parameter file, multiple mapping nodes can subscribe to the same GNSS topic.

### `map` versus `odom`

REP-105 requires `odom -> base_link` to remain locally smooth. Global
corrections, including GNSS and loop closure, should change `map -> odom`.

SKiD currently publishes the pose-graph estimate directly in the robot's
`odom` frame in
[`mapOptmization.cpp`](../src/mapOptmization.cpp#L1833-L1859). Loop closure
also rewrites the optimized key poses. Transform fusion then builds
`odom -> base_link` from the latest mapping pose plus the IMU increment. A
mapping correction can therefore make the nominal odometry edge jump, while
`map -> odom` is reserved for inter-robot map fusion instead of local global
correction.

This naming is usable for visualization, but it does not preserve the semantic
contract expected by navigation consumers.

### Inter-robot map fusion

The SOLiD map-fusion graph places robot index zero at identity with a tight
prior and estimates other robots from relative appearance/ICP constraints in
[`mapFusion_so.cpp`](../src/liorf-DiSO/mapFusion_so.cpp#L492-L640). It has no
ECEF state, datum, GNSS prior, or geographic separation factor.

This produces three failure modes for separated sites:

- without common geometry, the true relative transform is unobservable;
- before a transform is accepted, the identity `map -> robot/odom` broadcast
  falsely co-locates the robots;
- a perceptual false positive can align structurally similar but unrelated
  sites because geographic plausibility is unavailable and position search is
  disabled by default.

The map-fusion transport further labels its `Odometry` message with body-like
frame IDs in
[`mapFusion_so.cpp`](../src/liorf-DiSO/mapFusion_so.cpp#L1309-L1323), while the
consumer ignores those IDs and interprets the pose as `map -> odom`. The
transport therefore does not self-describe the transform it carries.

## Required transform authorities

Only one component may own each TF edge.

| Transform | Authority | Required behavior |
|---|---|---|
| `earth -> platform/map` | Geodesy/global alignment | Double precision; derived from an explicit WGS-84 datum or global estimator; absent while unknown |
| `platform/map -> platform/odom` | Local SLAM correction | May change after loop closure, GNSS, or relocalization |
| `platform/odom -> platform/base_link` | Continuous local estimator | High-rate, locally smooth, never receives pose-graph jumps |
| `platform/base_link -> sensor` | URDF/static calibration | Rigid and published once by the robot description authority |
| local pose `<->` UTM | Geodesy service/output | Zone- and hemisphere-qualified; not raw unqualified TF arithmetic |

If one site map is shared, replace `platform/map` with `site/map`; the odom and
body frames remain platform-specific.

## Proposed implementation

### 1. Make frame identity explicit

Add parameters or a structured configuration for:

- canonical `earthFrame`, default `earth`;
- fully resolved per-platform or per-site `mapFrame`;
- per-platform `odometryFrame` and `baseLinkFrame`;
- frame mode: `local_only`, `ecef_anchored`, or explicitly zone-scoped UTM
  output;
- datum latitude, longitude, ellipsoid height, heading convention, covariance,
  and datum source;
- altitude model, distinguishing ellipsoid height from mean-sea-level height.

Frame IDs should be resolved once and used consistently by TF, odometry,
paths, point clouds, markers, GNSS conversion, and map-fusion messages.

### 2. Separate continuous odometry from corrected mapping

Maintain two pose streams:

- a continuous local pose for `odom -> base_link`;
- an optimized pose for `map -> base_link`.

Compute the correction edge as:

```text
T_map_odom = T_map_base * inverse(T_odom_base)
```

Loop closure and GNSS update `T_map_odom`; they must not rewrite the continuous
`T_odom_base` history or make its live transform jump.

### 3. Add an ECEF/local-ENU anchor component

Use the GeographicLib dependency already present in SKiD:

- `Geocentric::WGS84()` for the datum's ECEF translation;
- local east, north, and up axes at the datum for the map-frame rotation;
- `LocalCartesian` for GNSS measurements expressed in the local map;
- double precision for all anchor and global-fusion calculations.

The first GNSS sample must not silently define a fleet datum. Use an explicit
surveyed/configured datum where available. If startup anchoring is intentionally
data-driven, publish its exact geodetic value and covariance as durable
metadata so replay reproduces the same frame.

### 4. Split local and global map fusion

The fleet graph should estimate `T_earth_map_i` variables. Factors may include:

- GNSS/ECEF priors with covariance;
- surveyed map anchors;
- accepted inter-robot registration constraints `T_map_i_map_j`;
- optional shared-site priors.

An inter-robot loop closure supplements a geographic anchor; it does not
replace earth. Disconnected graph components remain disconnected rather than
being initialized at a common identity. Descriptor searches can use ECEF
distance as a hard plausibility gate when the maps are known to be too far
apart to overlap.

### 5. Correct topic and message contracts

- Prefix relative GNSS topics per robot, while preserving explicitly absolute
  topic names.
- Honor the configured frame IDs instead of hardcoding `map`.
- Carry the NavSatFix covariance and status into the global measurement model.
- Use a transform-specific message or correct `Odometry.header.frame_id` and
  `child_frame_id`, and validate both in the consumer.
- Reject multiple authorities for the same TF edge.

## Acceptance tests

The frame work is not complete until these cases pass automatically:

1. **Single local robot, no GNSS:** `odom -> base_link` works while the map is
   detached or explicitly local-only.
2. **Single anchored robot:** a known datum produces a reproducible
   `earth -> map` transform and round-trips known geodetic control points.
3. **Two robots at one site:** independent odometry chains align in one site
   map without duplicate TF parents.
4. **Two distant robots without overlap:** both appear at correct ECEF
   separation; no SOLiD/ICP constraint is required or attempted.
5. **Two distant robots in different UTM zones:** positions reconcile through
   ECEF, and unqualified UTM arithmetic is rejected.
6. **UTM zone boundary crossing:** geodetic position remains continuous even
   when the projected zone representation changes.
7. **Unknown global position:** the local tree remains usable while
   `earth -> map` is absent; no identity global transform is fabricated.
8. **Loop closure:** `map -> odom` changes while `odom -> base_link` remains
   continuous.
9. **GNSS isolation:** each platform consumes only its own GNSS stream.
10. **Replay determinism:** an explicit datum reproduces byte-stable frame
    anchors across fresh processes.
11. **Precision:** local map points retain centimetre-scale resolution even
    when their earth position is millions of metres from ECEF origin.

## TestTrack interpretation

The successful 2026 TestTrack replay validates local scan matching, IMU timing
hardening, and the single-robot visualization tree. Its launch replays only one
LiDAR and one IMU, disables map fusion, and does not consume GNSS. It therefore
does not exercise `earth`, independent map datums, UTM zones, GNSS topic
isolation, or geographically distributed fusion.

That replay must not be cited as evidence that multi-site frame handling is
correct.

## References

- [REP-105: Coordinate Frames for Mobile Platforms](https://github.com/openrobotics/reps/blob/main/_posts/rep-0105.md)
- [GeographicLib `LocalCartesian`](https://geographiclib.sourceforge.io/2009-03/classGeographicLib_1_1LocalCartesian.html)
- [GeographicLib `CartConvert`: geocentric and local Cartesian conventions](https://geographiclib.sourceforge.io/html/CartConvert.1.html)
- [GeographicLib `UTMUPS`](https://geographiclib.sourceforge.io/2009-03/classGeographicLib_1_1UTMUPS.html)
- [`robot_localization` sensor-data frame guidance](https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/doc/preparing_sensor_data.rst)
- [`robot_localization` `navsat_transform_node` reference configuration](https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/params/navsat_transform.yaml)
