# v292 — Gamepad Prepare after receiver reconnect

2026-10-18. Frontend fix; no movement testing or control parameter changes.

## Cause and evidence

The gateway accepted Prepare and then received `gamepad_disconnected` from the
page repeatedly at approximately 12:42–12:43. The user saw a controller name;
reselecting it in Robot-Controller Pairs immediately restored Prepare. The
upstream detection hook cleared assignments only when *all* controllers were
absent. A stale slot therefore survived a reconnect between polls or while
another pad remained visible. The E6 reader continued polling that absent slot.

Raw excerpts and offline reproductions are in `evidence/prepare-v292/`.
This identifies the stale-binding software path; the receiver's exact OS slot
transition was not captured directly.

## Change

- Validate assigned slots against every detected list, clearing only vanished
  selections and retaining robot/group configuration and valid selections.
  Reuse upstream auto-selection for the available controller.
- Refuse Prepare when the selected pad is missing or disconnected, with a short
  visible explanation instead of successful preparation followed by cancellation.
- A changed device identity or disconnect event stops the old input. The new
  device must be centered and explicitly prepared. Pending socket preparation
  cannot carry over to another device or survive a page blur/Stop.
- Same-slot reconnect events are handled even between polling ticks. The
  listener is removed on unmount. All input continues over the existing socket.

Formal sources: `frontend/gamepadSelection.ts`, `frontend/useE6Control.ts`.
Upstream integration: `patches/0013-e6-gamepad-reconnect.patch`, on phosphobot
`a5de051197c879e3c685d1362f649ce54ee47a3c`. No dependency or UI theme added.

## Validation

- Before change: 4 failed / 70 passed focused tests, including stale selection
  and missing/replaced device preparation cases.
- First full run: 12 failed / 187 passed; synthetic fixtures claimed readiness
  after switching source without explicit Prepare. They now perform explicit
  centered Prepare before simulating its acknowledgement. Physical/synthetic
  comparisons also assert active nonzero mapping, avoiding neutral-equals-neutral
  false passes. Failure logs are preserved.
- Final suite: **203 passed**, 13 test files. Includes sparse slots, another pad
  remaining, valid manual selection, all-pad disconnect, replacement, same-slot
  reconnect, pending socket recovery, listener cleanup, existing input isolation,
  keyboard/gamepad mapping and recording lifecycle.
- TypeScript and Vite production build passed (2514 modules).
- All **13 patches** apply in an isolated Git index at the fixed upstream HEAD;
  all 20 patched files match the generated checkout, and all 30 extension files
  match formal sources byte-for-byte. The working index and gitlink were not reset.
- Existing test-suite `TimeoutNaNWarning` was retained; no test failed on it.

Browser URL security policy rejected automatic access. No alternate browser
surface or scraping was used. This is offline DOM testing, not a claim of fresh
browser visual QA or physical receiver reconnect validation of the new build.

## Publication and rollback

Old served assets and source hashes are retained in
`output/prepare-v292/baseline`; the validated package is in
`output/prepare-v292/release`. Use
`teleop-platform/scripts/switch_gamepad_reconnect_v292.py --check`, then run
without `--check` only after Stop, leaving Control, and fresh idle feedback.
The helper publishes hashed assets before atomically replacing `index.html`,
keeps older assets, and neither restarts the hardware owner nor opens a control
connection. Rollback uses the same helper with `--rollback --check`, followed
by `--rollback` under the same stopped conditions.

Publication status is recorded separately in `publish-result.json` and the
current STATUS entry. At the initial preflight the user was operating
(`ready=true`, `stationary=false`), so no publication occurred then.

The v291 operator remains `20261018-123541-e9850927`, entry
`output/startup_operator_291.py operator`, stopfile
`output/startup_operator_291_operator.stop`. Do not infer current readiness,
pose or gripper opening from the earlier startup snapshot. Goal remains paused.
