# v294 — Recorded slowdown corrections

2026-10-18. Candidate G published after offline qualification. Fresh online
checks and the user-operated gamepad translation/reversal/gripper test passed.
This physical run contains no rotation or tool-frame input. Full Goal remains
paused. No Git commit was requested or made in this turn.

The v293 audit found three independent defects: unchanged-target CAN exchanges
disqualified directional retreat, frozen adaptive contact failed a target-error
stationarity test, and the base/Link2 convex hulls filled source CAD concavities.

## Candidate behavior

- G3 carries a fresh native hold classification independently of transaction
  progress. Same-target inflight exchanges keep every geometric commitment.
  Actual movement, a new target, unknown/stale feedback, contact escape and
  dropped/fault states do not acquire a stationary-retreat classification.
- Frozen closing contact can request the existing retreat proof. The proof also
  spans the complete committed aperture range and raw feedback ±1 count. Pairs
  affected by that aperture uncertainty must stay beyond proximity over the
  whole arm segment; otherwise ordinary slowdown remains. The separate full
  arm/finger collision sweep, deadlines and consumption checks remain active.
  There is no automatic target chasing, release, increased force or clearance.
- Only base_link and Link2 body colliders are refined. CoACD 1.0.14 chooses a
  partition; each complete original triangle is re-enclosed in an exact convex
  hull, Minkowski-expanded by a 0.1 mm cube. Approximate CoACD outputs are not
  accepted as collision boundaries. Original visuals, attachment boxes,
  transforms, ACM, joint margins and TCP remain unchanged.
- Source `scripts/refine_workspace_body.py` derives the new version from the
  saved parent model. Model ID:
  `3b4003e47dc241273476c7a01a7102963d06ed59767ddd010afdaeeba72a72d7`.
  Rebuilding produced a byte-identical manifest. The original hull proofs are
  retained as parent provenance; `body_refinement` names the active replacement
  parts, which tests match to the actual URDF.

## Model and software verification

- `evidence/slow-v294/tests-first.log`: 140 targeted tests passed.
- `backend-all-first.log`: 984 passed, 8 skipped. Later protocol-pin test: 23
  focused tests passed (includes one new test). Existing warnings retained.
- `staged-model-tests.log`: 57 tests passed against the staged model, including
  display/control geometry identity and unchanged attachment configuration.
- Serialized STL containment verifies all 191,762 original triangles. Minimum
  outward reserve remains approximately 0.1 mm after float32 serialization.
- 2,943 deterministic whole-range, recorded and nearby configurations:
  candidate 529 collisions, source reference 512; no source collision was
  missed by the candidate in this sample. This is not a universal proof or
  physical clearance calibration.
- Recorded base/Link2 clearance: parent hull 6.012 mm, candidate partitions
  17.838 mm, original CAD 19.474 mm. No physical distance measurement is claimed.
- 262,145 deterministic workspace samples; 155,222 collision-free samples;
  136,747 distinct 10 mm voxels. Every retained witness rechecked with zero FK
  discrepancy and no collision. Parent had only 65,537 samples; counts must not
  be presented as a fourfold workspace enlargement.
- Dense results are archived separately. The display map uses the original
  65,537-sample budget: 37,516 verified voxels, about 6.75 MB versus the prior
  37,044 / 6.67 MB. The 24.6 MB dense witness set is not sent to the webpage.
- `body-3d-comparison.png`: independently rendered same recorded pose, two
  views, parent/candidate collision overlays on CAD. This is not webpage QA.
- New camera snapshot inspected. The cup is in front of the fingers. Wrist
  view does not establish whole-environment clearance or cup reachability.

## Final candidate G qualification and publication

- Binary: `7e7d30635e011c26ba8aed2d94ec02876986054ae0cf25180004273e8146e6fa`.
  The exact deployed native sources, test harness, model and binary hashes were
  checked against the qualification outputs. The isolated job exited 0 and
  restored the original CPU governor before publication.
- Original native suites: 55 cases / 2,085 frames and 45 cases / 1,383 frames.
  Deliberate collision/singularity rejection cases still include 67.56–79.32 ms
  callbacks that are rejected by the unchanged deadline. This is not a
  whole-workspace worst-case execution guarantee.
- G3 matrices: 22 cases / 2,326 frames on the parent model and 70 cases / 3,622
  frames on the refined model. The latter includes 48 positive/negative axis
  cases in base/tool frames, translation orientation and rotation TCP checks.
  Five independent cold-process replays also passed (167 frames each).
  These seven runs contain 6,020 active frames, zero budget exhaustion or
  protected direction stops, P95 callback 25.834 ms and maximum 38.743 ms.
  Unknown/moving/escaping fingers still cannot acquire a stationary boost.
- Final independent policy validation: 384 cases with 53,868 dense checks of
  accepted paths, plus 402 dense checks of the two saved boundary-exit targets.
  Compound-distance optimization matched 34,128 exhaustive distances exactly.
- Final focused backend checks: 120 passed; complete earlier backend run:
  984 passed / 8 skipped. The pending v292 controller-reconnection UI fix is
  included: 203 frontend tests, production build, 13-patch / 30-extension
  reproduction passed. `git diff --check` passed (existing CRLF notices remain).
- Reviewed `slowdown-comparison.png`, `compound-timeout-comparison.png` and
  `clearance-transition.png`. Same-target polling speed notches, frozen-contact
  slowdown and the stale-monitor boundary-exit notch are absent in these
  corrected replays. They show solver candidates with synthetic feedback,
  **not measured physical robot velocity**. CAD/collision overlays were reviewed
  separately in `body-3d-comparison.png`.

The checked bundle switched at 14:42:47 local time. Previous directories are
retained with suffix `20261018-144247-493841`, in addition to immutable
`output/slow-v294/baseline`. Only `cpp_runtime_manifest` changed in workstation
configuration. TCP, payload, Force255, speed/ramp settings, bounds, ACM and all
motion deadlines retain their prior values.

Owner at publication (still the same owner after the user test):
`20261018-144306-c02ecf61`, launched by
`output/smooth_operator_294.py operator`; stopfile
`output/smooth_operator_294_operator.stop` is absent. Synthetic input is disabled.
Initial fresh 20-state observation and `live-check.json` confirmed unprepared/stationary,
recording idle, Mode5 / Enable1 / Error0 / Collision0, command ID8138, payload
0.750 kg / COM(0,-14,46), matching runtime/model hashes and valid video.
Gripper attached by observation at raw203–204, native velocity0, G3 hold1;
`sent_raw=null`, with no initialization position command. Current measured
condition number is approximately 33.478; actual singularity slowdown remains
possible and must not be described as entirely removed.

Startup HTTP 503 responses were retained in the normal logs while the remote
gateway/model loaded; the later fresh checks above passed. New camera snapshot
`startup-color.jpg` was inspected: the cup remains near the fingers. It does not
prove a collision-free task path. No motion, Prepare, Enable, clear-error or
payload-write command was issued during this publication.

The user subsequently reported “测试完成，操作顺畅”. The browser tool's
local-URL security refusal was not bypassed by another UI or control channel.
The user operated the formal page; automated page visual QA is not claimed.

### User-operated physical test

Evidence: `evidence/slow-v294/manual`. The fixed slice includes 1,810.637 seconds
of observation, mostly idle before/after the approximately 63.343-second arm
operation window. Original files remain intact. Frozen byte boundaries and
compressed/uncompressed SHA256 values are retained in `capture.json`.

- 220,752 original 1,440-byte SDK frames were decoded and checked: no sequence
  gaps, device errors or collision flags. Modes were enabled idle5 and running7.
- All 7,324 input packets identify `gamepad`. Exactly one Prepare occurred.
  Ready stayed true until explicit `operator_stop`, followed by `page_blur`;
  there was no `input_stale` or spontaneous Prepare loss in this capture.
- All 1,032 submitted motion identities match 1,032 accepted zero-code replies.
  All are **base-frame translation**. Nonzero gripper inputs also occurred;
  no angular or joint-mode input occurred, so this run must not be presented
  as physical rotation/tool-frame acceptance.
- All 1,032 submitted paths have collision scale1 and no geometry-budget
  exhaustion. 924 distinct active geometry observations have no model
  rejection; callback P95 6.714 ms, maximum 13.969 ms against the unchanged
  42 ms deadline. These are recorded observations, not a claim to have sampled
  every internal native result.
- True singularity handling remained active: conditions approximately
  33.477–51.070, with status1/3 (approach/retreat). The previous erroneous
  collision-scale toggling is absent from this physical record.
- Eight Stop requests have eight accepted zero-code replies. Command reply
  round trips were 618.844–702.292 ms; these are not a measurement of the
  exact instant physical motion ceased and are not described as millisecond
  stopping. Final fresh state is stationary and unprepared.
- 122 gripper target-permission waits each resumed; P95 99.031 ms, maximum
  251.242 ms. These waits remain in the report rather than being labeled zero
  gripper delay. They did not create a model rejection or cancel Prepare here.
- Actual TCP component peak-to-peak range was approximately
  (46.891, 32.697, 33.066) mm; maximum displacement from the first submitted
  sample was 43.629 mm. This is the observed user path, not all-workspace
  acceptance or proof of cup-grasp reachability.

`actual-curves.png` was visually reviewed. It shows measured TCP motion,
50 ms finite-difference velocity (faint), a 0.25-second visualization filter
(solid), collision scale, native callback duration and Jacobian condition.
Filtering is not used as a control permission or as proof of zero jitter.
`event-review.json` independently matches every submission to its accepted
reply. The first analysis incorrectly assumed an offline-only `cpp.active`
field; its superseded outputs are explicitly retained as `schema-first`.
Final analysis uses actual source intents and active model observations.

Final new 20-state check: same owner, unprepared `page_blur`, stationary, idle,
Mode5 / Enable1 / Error0 / Collision0, command ID9170, gripper held raw86–87
(native fraction approximately33.7%, sent87), video valid. No new assistant
motion, Enable, clear-error, parameter change or gripper initialization was
issued during evidence collection. The Goal remains paused.

Rollback: normally release the owner with its stopfile; verify clean exit and
closed service ports, then run
`teleop-platform/scripts/switch_smooth_teleop_v294.py --rollback --check` followed
by `--rollback` using the existing connection provider. This restores the saved
binary pin, four matching backend files, assembly, map and UI together. Do not
start a mixed old-binary/G3-source session.

## Retained development failures and earlier session

The old owner `20261018-123541-e9850927` exited normally after 20 fresh states
confirmed unprepared/stationary/recording idle. Remote camera/gateway exited 0;
CPU and Wi-Fi policy restored. Local media exit 1 retained; cleanup errors empty,
overall clean true. No enable, clear-error or motion command was issued here.
Baseline is `output/slow-v294/baseline`; do not overwrite it.

The candidate is being built and exercised inside the existing X5 ROS rootfs
with isolated network/mount namespaces and a project session lock. First build
failed because PYTHONPATH replaced ROS paths; diagnostic retained in attempt-a.
Retry preserves ROS paths. No system ROS package was upgraded or installed.
The first native build and original 55/45 regression suites completed (attempt-b).
A subsequent review added aperture uncertainty to the proof-cache identity;
the final build and rerun are in progress. `hold-policy-parent/candidate.json`
record 384 independent PC policy cases, with 15,075 dense path checks for
accepted retreats. `cache-regression.log` verifies changed/malformed uncertainty
cannot reuse an unrelated proof (201 additional dense checks).

The next native run (attempt-c) exposed an additional performance regression:
contact at 15 mm/s reached a different close body pair. Enumerating every
compound shape pair cost 97–125 ms in the initial distance query, producing
84 protected direction stops. That build was not published. The complete
failure frames and candidate package are retained under `attempt-c` and
`release-attempt-c`.

The next candidate orders compound shape pairs by FCL's enclosing RSS lower
bound and omits a pair only when that bound exceeds an already measured exact
minimum. It retains the same geometry, ACM and thresholds. An independent
all-shape enumeration comparison over 432 recorded, nearby and random poses
checked 34,128 pair distances with zero numerical difference. X5 native timing
and regression qualification of this change is still running.

Attempt-d preserved the correct distances but still failed the same native
contact case: 44.2 ms median / 47.1 ms maximum callback, with 84 protective
stops. It was also withheld from publication. Independent compound queries
now reuse the existing seven workers plus caller and join before any permission
is returned. Transforms are materialized on the caller, BVHs stay private and
immutable, and atomic minima only skip pairs with an already-proved larger
lower bound. Sample-level parallel batches do not recursively dispatch to the
same pool. The 432-pose / 34,128-pair exhaustive comparison again has zero
distance difference. Attempt-e starts with this exact failed contact case
before the full regressions. No deadline or protective threshold was increased.

Attempt-e reduced that failure from 84 stops to one cold-entry stop, but its
42.303 ms callback still failed acceptance and was retained without publishing.
The failed callback spent 20.684 ms in initial distances and 13.850 ms in two
sample queries. The next candidate initializes search upper bounds using real
triangle witnesses reached through the native RSS tree, then performs exact
FCL traversal. For one/two path samples the existing workers handle compound
parts; a proven approach can reject after the first sample. Larger batches keep
sample-level parallelism. This does not extrapolate a previous distance or
reuse a permission. Five fresh-process replays of the cold-entry case precede
the full matrix. The final PC 34,128-distance comparison remains zero-difference;
the 384 policy cases and 15,075 dense checks retain the same decisions.

Candidate-f passed all five fresh-process failure replays: 600 active reversal
frames, zero stops/budgets, callback maximum 35.176 ms. Visual review of its
velocity curves nevertheless found a notch after exiting proximity: current
clearance was 10.166/10.546 mm but the asynchronous monitor still supplied
0.3795 scale. Candidate-f was not published. A further correction requires a
current whole-interval separation proof, or the same continuous collision
proof and monotonic distance samples for every remaining pair, before reporting
`clearance_restored`. This covers stale monitor transitions while retaining
actual collision, aperture uncertainty, singularity and timing protections.
Its fresh native regressions and new dense verification are still pending.
The two retained bounded targets at the notch now return `clearance_restored`;
402 independent dense checks confirm collision-free paths. The first increases
the limiting distance from 10.166 to 10.546 mm, and the second has a complete
whole-interval separation certificate. These are the saved candidate-f bounded
targets, not a fabricated reconstruction of its discarded pre-scaling trial.

Browser local-URL security policy prohibits automated page access. No native UI,
CDP or alternate browser bypass is used. Formal webpage physical operation and
page visual QA therefore remain outstanding; offline render/solver tests cannot
be labeled real hardware or physical wireless-receiver acceptance.

Physical coverage in this delivery is the recorded user gamepad base-frame
translation/reversal and gripper operation above. Rotation/tool-frame behavior
has offline coverage here; this user test did not exercise those inputs.
Automated page visual QA, full workspace/full-speed testing and elimination of
all wireless or controller timing tails are not claimed.
