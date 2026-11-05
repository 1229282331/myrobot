# v293 — Repeated slowdown during cup approach

2026-10-18. **Diagnosis only; no backend, collision model or runtime change.**
The user confirmed manual controller reselection restored Prepare, then reported
frequent slowdowns during Cartesian operation. The validated v292 frontend fix
remains unpublished so these symptoms cannot be attributed to its deployment.

## Evidence and current state

- Existing owner `20261018-123541-e9850927` retained. Read-only HTTP state,
  existing session camera snapshots and a bounded SSH audit prefix only.
  No Prepare, enable, clear-error, motion, gripper command or service restart.
- Captured 268,412,650 complete audit bytes, ending at the fixed offset in
  `evidence/slow-v293/provenance.json`; original remote file unchanged.
  3427 submitted motion records and 8248 cycle records cover the recent user
  operation and subsequent idle interval. The submitted count need not equal
  confirmed count in a cut byte window; this is not a lost-ack claim.
- Fresh image shows the white cup in front of the fingers. Wrist-camera view
  does not show the base/shoulder clearance and alone does not establish a
  continuous fixed-orientation approach path.
- Initial and final fresh windows: unprepared due to `page_blur`, stationary,
  Mode5/Enable1/Error0/Collision0, command ID8138, recording idle, gripper held
  at native raw204, current feedback valid, camera valid. Existing payload,
  TCP, model and solver hashes retained. New motion must use new feedback.

## Confirmed findings

### Collision geometry overstates proximity at this pose

Current production model `b4e1f8bc…` identifies **base_link / Link2** as the
nearest arm-motion-dependent pair. At the captured joint state:

| Geometry used for this pair | Surface distance |
| --- | ---: |
| Production conservative convex hulls | 6.01225 mm |
| Original official CAD surfaces, same transforms | 19.4737 mm |

The diagnostic changes only the two mesh filenames in an offline URDF. Both
queries use the same existing MoveIt/FCL probe and assembly transforms. The
production URDF and collision matrix were not modified. The convex hull fills
concavities and includes conservative expansion, bringing this pose into the
10 mm proximity slowdown region. Its calculated multiplier is about 6.36%.
The last active command used 6.735%, at an earlier measured pose.

This is a **nominal model comparison at one pose**, not a physical measurement
or certification of an alternative model. Other poses still need collision
coverage and timing validation. Source/model details:
`source-pair-comparison.json`, both `*.urdf.distances.log`, `distance.json` and
the diagnostic URDF in `evidence/slow-v293/`.

### Same-target CAN transactions incorrectly suppress retreat

`servo_cpp_adapter.cpp` requires `grip_inflight < 0` before evaluating a
directional retreat. A routine transaction with the *unchanged* position target
temporarily sets inflight to that same target. This is transport state, not
evidence that a new finger motion was requested.

Captured epoch149 contains a clear example: sent/requested target204 throughout,
feedback203–204, no gripper-rate input, fresh `state=0/at_target` and zero velocity
replies. While the arm is retreating, the scale switches between 1 and roughly
0.03–0.07 as inflight changes between -1 and204. In the first 2.25 seconds, six
submitted frames were rejected for this reason. The singularity condition
remained below30 during those dips. Across the captured operation, 124 submitted
frames have this same unchanged-target pending combination.

`retreat-stutter.png` plots requested/observed Y velocity, selected scale,
pending transactions and singularity condition. Observed velocity is derived
from sampled TCP positions with a 0.25-second filter, not raw SDK velocity.
It illustrates the association; it does not attribute every velocity fluctuation
solely to this branch. `output/plot_slow_293.py` reproduces it from retained data.

### Normal adaptive contact is also conflated with motion

Earlier in the captured operation, gripper feedback remains raw131, normal
contact state2, while the deliberately frozen grasp target is123. The arm
retreat gate requires actual and sent positions to differ by at most one count,
so it classifies normal grasp contact as `gripper_not_stationary` continuously.
1075 submitted arm frames have this combination with zero gripper-rate input;
1040 sampled contact cycles span70.02 seconds in the retained window.

This needs a contact-aware policy, not a change to adaptive grip force or an
automatic move to the unreachable target. Nominal linkage cannot certify
individual passive finger deformation under load; retained target ranges and
independent arm/finger collision checks must still be respected.

## Other observations

- 2340/3427 submitted frames report collision/proximity status4; the rest include
  no-warning and singularity approach/retreat statuses. These counts are selected
  command samples, not full-session duration percentages.
- Current condition is about33.48 (slowdown threshold30, stop60). Singularity
  contributes in some portions, but cannot explain the low-condition dips above.
- Captured Prepare transitions are only the user's explicit Prepare and two
  page-blur cancellations. No `input_stale` or `model_direction_blocked` event
  appears in this prefix. Stop requests alone include ordinary input release
  and must not all be called unexpected disconnects.
- Current relevant pair is at the base/second link, not the cup or fingertip
  envelope. Environment objects, including the cup/table, are not in the model.

## Targeted correction scope

1. Separate an unchanged-target CAN exchange from a new geometric commitment;
   retain feedback freshness, target bounds, dynamic sweep and deadline checks.
2. Handle normal stationary contact independently from outstanding opening
   motion; do not use target error alone to decide whole-arm retreat eligibility.
3. Replace the overfilled body hull where justified using source-based geometry
   or validated decomposition. Preserve collision coverage and qualify actual
   X5 cycle time before changing the live model.
4. Regress these real captured trajectories, fixed-target CAN polling and
   contact holding together. Independent unit tests alone missed this interaction.

Do not fix this by simply enlarging timeouts, shrinking every envelope, disabling
collision checks or raising the singularity thresholds. No claim that the cup
has been reached, that new code is deployed, or that all slowdowns are fixed.

## Deferred v292 publication

203 frontend tests and final build passed. All13 upstream patches reproduce at
the fixed pin and30 extension files match. Final package is updated to
`index-D14c2Q1P.js`; baseline remains intact. Neither static publication nor
runtime restart occurred. See `GAMEPAD_RECONNECT_V292.md` and
`scripts/switch_gamepad_reconnect_v292.py` for stopped publication/rollback.
Complete Goal remains paused; no Git commit was requested in this diagnosis.
