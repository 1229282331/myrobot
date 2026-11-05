# v299 Geometry deadline recovery

Status: candidate B is published. Isolated qualification and limited formal
webpage physical checks are complete. Remaining model proximity slowdown and
video pause behavior are explicitly retained below.
No new Enable, fault clear, payload change or gripper command in this revision.

## Incident and timing policy

The current operator reported that every direction became unusable after the
geometry timeout warning. The retained 96 MiB tail of session
`20261018-221557-3ff8629a` contains 106 distinct active model results: 102 callback
deadline rejections and four valid results. Condition numbers around 18.66 are
below the singularity slowdown threshold of 30. Initial retreat distance queries
take roughly 30–40 ms on X5, followed by a repeated gripper query around 16 ms.
The later unprepared state follows an explicit operator Stop and page blur;
the old directional warning obscures that lifecycle reason.

Current model distances at the captured pose are approximately 5.270 mm between
Link2 and `op_hand_wide3_Link`, and 7.488 mm between Link2 and
`op_hand_wide_loop_Link`. These are nominal model distances, not physical gap
measurements. Gripper aperture uncertainty still prevents some retreat boosts.

The 42 ms callback limit is an engineering allocation inside a 50 ms cycle, not
an immutable physical constant. Removing it or increasing it alone can admit
stale results or increase command latency. Candidate A retains 37/42/50 ms and
the 100/250 ms feedback/input gates while reducing duplicate computation. A
different timing allocation requires whole-pipeline timing and stop validation.

## Candidate A

- Reuse exact link-pair surface separation values computed by the retreat check
  as conservative lower bounds in the subsequent gripper broad phase.
- Reuse is limited to the same callback, model identity, all actual joint/aperture
  coordinates and collision matrix, with a successful original-ACM anchor
  collision check. Attached bodies, world objects, padding/scaling changes,
  changed poses/matrices and reset invalidate reuse. No target permission is
  cached. Unmeasured or uncertified pairs still use the original FCL query/sweep.
- Keep original geometry, ACM, 40 mm sweep travel and 1 mm spacing, aperture
  commitments, numerical margins and deadline checks.
- Display the current unprepared/session-pause reason ahead of an old model
  direction warning. Actual hardware failures and Stop failures still stop.

## Evidence so far

`evidence/geometry-recovery-v299` contains the incident tail, original current
state, pre-release states and PC witness set. Candidate A PC test: 60 cases,
568 independent dense checks, 59 certified intervals; median gripper certificate
time 1.441 ms before / 0.136 ms with reuse. PC timings do not establish X5 timing.
Fifty targeted backend tests pass, including geometry timeout followed by Stop,
disconnect, feedback failure, failed Prepare and existing stream recovery.

The old owner exited cleanly after 20 fresh unprepared/stationary/recording-idle
states. Camera/gateway exited 0, CPU and Wi-Fi policies restored; local media
exit 1 with empty cleanup errors is retained. The actual deployed binary and
pin, gateway source and local configuration are saved in
`output/geometry-recovery-v299/baseline`, with hashes. X5 candidate A builds under
the existing exclusive session lock in an isolated mount/network namespace,
single compiler job and original 1,900,000 KiB memory limit.

Pending: X5 original/candidate incident replay, positive/negative axes, tool/base,
rotation/position-assist/joint modes, release/restart, aperture/hold state changes,
pipeline regression, controlled publication and fresh formal-page acceptance.
No claim of resolved full-workspace latency or physical retreat validation yet.

## X5 A and PC B update

X5 reproduced 80/80 baseline active callback timeouts at the original pose.
The two test directions have median whole callbacks of 52.278/52.162 ms.
Reuse-only A (8796813e...) permits 79/80 with medians 38.461/38.349 ms;
one 42.974 ms result is rejected. A has not been published.

B prioritizes exact uncertainty-affected pair queries and stops the retreat
assessment once a measured distance within 10 mm proves the original stationary
boost requirement cannot pass. Incomplete results are explicitly rejected as
retreat permission. Measured same-anchor pair separations can still be reused by
the following unchanged collision sweep. PC parity: 144 cases, 21,909 independent
dense checks, all decisions/reasons equal to the actual deployed baseline.
This includes five actual collisions, 25 uncertain-gripper cases, three retreats,
three approaching-pair refusals, 106 clearance-restored and two no-improvement.
PC B same-anchor tests: 60 cases/568 dense checks; median initial retreat query
1.343 ms plus subsequent gripper query 0.423 ms. X5 B is compiling; no new timing
threshold or real motion has been introduced.

## Final B qualification and publication

Binary `6314bdc0ff22586fc42c5be4e38b96a5f0e54380e9fd5e27e1a44a6785a94633`:
- X5 stuck-pose smoke: 80/80 active targets accepted, zero timeouts. Translation
  callback median/max 21.237/24.456 ms; rotation 21.156/21.764 ms.
- X5 direction/mode suite: 41 cases, 2,190 main active frames, all accepted; zero
  timeout or unresponsive case. Release/restart frames also meet the deadline.
- Four 30-second real worker/IPC plus dual-journal runs (synthetic 100 ms plant
  delay): 524/522/526/595 accepted, zero activity rejection, no writer loss or
  worker error. Maximum consumption age 38.943/39.128/49.978/46.310 ms.
- Old-mode comparison: 24 cases/720 frames, exactly equal joint targets and
  statuses. All isolated jobs exited 0 and restored CPU governor to schedutil.
- Full backend: 1,066 passed, eight skipped. No frontend source change.

The initial publication check rejected a pin hash mismatch: the staging script
had reserialized the baseline JSON. Semantic contents were exactly equal and
the remote raw bytes matched the original workstation pin hash. The raw file
was restored to the baseline, the serialized copy retained, and manifest hashes
updated. No production binary/configuration difference was ignored. The repeated
strict check passed and the package switched normally while no owner was active.

New test owner `20261018-232951-2ffea40d`: 20 fresh samples show unprepared,
stationary, recording idle, Mode 5/Enable 1/Error 0/Collision 0, ID 21808 unchanged;
payload matches and gripper observed ready at raw 80, sent target null. No Enable
or gripper initialization was sent. The remaining model proximity limit is not
removed by this computation optimization.

## Physical webpage checks and remaining limits

Test session `20261018-232951-2ffea40d` used the formal webpage at 0.2x:
10 keyboard segments (XYZ positive/negative, Rx positive/negative, assisted Z
positive/negative), two six-round synthetic gamepad reversals (base Z and assisted
tool X), and one post-release restart. This does not validate the physical receiver.

- 64,212 original SDK frames: no missing sequence, alarm, collision or command
  acknowledgement issue. 789 commands submitted and all 789 acknowledged.
- 733 unique active model results: no geometry timeout, direction block or
  solver-stream pause. Callback median/P95/max: 23.076/26.067/34.003 ms.
- Nine old-candidate rejections occurred after neutral input; no rejected active
  command was permitted. All Stop receipts succeeded.
- From gateway receipt of neutral input to Stop send: 1.379–9.442 ms. Stationary
  window confirmation followed in 797–968 ms; this includes the configured stable
  window and is not an end-to-end braking-time measurement.
- Maximum TCP displacement from test-window start: 0.2983 mm. Largest single
  segment orientation change: 0.0923 degrees. These are limited local checks.
- Position assist generated no angular assistance at this pose (`holding_orientation`);
  these trials do not add physical validation of automatic wrist adjustment.

All 733 active results retained `gripper_proximity_uncertain`: model velocity
scale 0.03179–0.03916 (median 0.03757). Arm/finger proximity and aperture uncertainty
still prevent retreat acceleration. This revision restores fresh directional
targets; it does not claim normal speed or fully resolved usability at this pose.
The distances are nominal model distances, not measured physical gaps.

Two `preview_stale` Prepare cancellations occurred during idle mode-selection,
after movement had already released. One showed paused video and required page
reload; the other recovered with visible video and explicit Prepare. Root cause
of this browser/video behavior is not claimed fixed. No Prepare cancellation
occurred during the 13 active test segments. All transitions remain recorded.

The four actual-feedback plots and isolated A/B timing plot were inspected.
Evidence hashes, failed A attempt, unrounded observations and raw frames remain
under `evidence/geometry-recovery-v299`; analysis scripts are in `output/*299.py`.

## Delivery and rollback

Test owner exited cleanly: camera/gateway 0, CPU and Wi-Fi policies restored.
Local media exit 1 with no cleanup errors is retained. Daily owner is now
`20261018-234141-b1b87157`, launched by `output/geometry_operator_299.py operator`
(exec 91191); stopfile `output/geometry_operator_299_operator.stop` is not present.
Acceptance/synthetic input is disabled. Twenty fresh final samples show
unprepared, stationary, idle, Mode 5/Enable 1/Error 0/Collision 0, command ID 22597
unchanged, matching payload and observed gripper ready/raw 80, sent target null.

Formal Gamepad page is open at Position assist/Base/0.2x with live 3D and RGB near
20 fps. Physical controller is not currently detected; wake it before Prepare.
Screenshot: `evidence/geometry-recovery-v299/final-platform.png`.

TCP 151.4 mm, model `3b4003e4...`, assembly B, payload 0.750 kg/COM (0,-14,46) mm,
Force 255, joint margins, smoothing and 37/42/50 ms timing are unchanged.
No new Enable, fault-clear, payload write, gripper move or cup-grasp action was
performed. Full Goal remains paused. This revision is not Git-committed.

Rollback: after Stop, fresh stationary/unprepared/recording-idle checks and normal
owner release, run `teleop-platform/scripts/switch_geometry_recovery_v299.py`
with `--rollback --check`, then `--rollback` using the existing SSH connector.
Actual baseline/release files and hashes are in `output/geometry-recovery-v299`.
