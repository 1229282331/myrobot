# Dynamic gripper bounds — v269

Status: v269 dynamic model released; offline verification and bounded formal-page gripper tests passed. Ready for user experience acceptance. Full platform Goal remains paused. The user restarted RDK, E6 and PC during testing; pre-restart session state is historical.

## Geometry and meaning

- User-confirmed mount B: gripper rotated −90° around its installation Z; R4 and Gemini E unchanged. TCP remains 151.4 mm.
- Each of the eight moving official CAD links owns one local oriented box. Its pose follows the actual CAN aperture through the shared 256-entry nominal unloaded linkage table. Sent/requested values never stand in for measured aperture.
- Every local box includes 5 mm per face. All 453,948 source vertices are covered. The analytical displacement bound between neighboring raw-count samples is at most 0.516025 mm.
- Actual aperture is telemetry; individual linkage angles and passive fingertips remain nominal, not calibrated independent joint measurements or a contact deformation model.
- When aperture telemetry expires after 200 ms, freeze the last displayed fingers and use the separately generated full-stroke fallback for collision. That fallback encloses every dynamic box over the nominal stroke, including its reserve, plus 1 mm for interpolation. It is not the normal known-aperture collision shape.

## Execution

The official MoveIt/FCL adapter receives `G2 actual sent inflight next_proposal`. It checks the arm segment against already committed gripper movement first, then the proposed opening interval. A bounded geometry lock coordinates CAN commitment with the existing ServoJ writer; FCL work and CAN exchange do not hold that lock.

The proposed CAN command uses a four-count batch; the final batch can contain up to seven counts to preserve the existing endpoint remainder behavior. Release discards unsent intent. A denied direction holds the last sent target and can reverse without restarting the session. Speed, force, TCP, payload, arm filtering and independent Stop remain unchanged.

Geometry work has a 30 ms budget inside the existing 50 ms solver period. Exhausting the proposal budget denies that proposal; an independently checked arm segment remains usable. No queued calculation catches up later, and permit timestamps come from the request, not its completion.

## Findings retained during development

1. A full unbounded proposed opening combined with a 30 mm/s ideal arm input took up to 510 ms. The bounded next-command proposal replaces that path. The adversarial full-range input is still retained and now denies the new aperture within budget.
2. The old full-stroke boxes were not generated from the new dynamic boxes. Regenerated the fallback from the actual dynamic colliders and added containment evidence.
3. MoveIt group filtering uses `getUpdatedLinkModelsSet()`. A fixed fallback link listed in the group is not necessarily in that set. A minimal FCL reproduction shows the whole robot finds `Link2/op_unknown_sweep`, whereas the gripper group misses it. Unknown-state queries now check the whole robot; known-aperture queries use the moving-link group. The two failed qualification runs are retained.
4. A formal lower-limit test exposed a quantization mismatch: 15% rounded to raw 38, then the geometry adapter expressed it as 14.90%, which the unchanged CAN encoder correctly rejected. The gripper faulted and the viewer/control switched to conservative fallback. Quantization now stays inside the configured range (raw 39 through 153, about 15.29–60%). Six regressions use the real CanDevice encoder with a simulated bus, with/without the geometry guard at all three rate profiles. The failure audit and state are retained in `before-endpoint-fix/` and `lower-limit-fault-state.json`.

Evidence: `evidence/dynamic-gripper-v269/` at repository root. The model fingerprint and numerical results are in its JSON reports; failures are not removed from the evidence history.

## Verification ledger

- 217 relevant backend tests passed, including real-encoder endpoint quantization, in-flight provenance, stale/malformed permits, arm/CAN commitment, blocked direction reversal and independent Stop. An obsolete assembly fixture expecting 11 static boxes on one link was updated to verify 3 fixed boxes plus 8 independently jointed boxes; its initial failure is retained.
- 170 distinct frontend tests passed; production dashboard build passed. Gripper freshness is independent of arm freshness; display interactions do not create a second control connection.
- Final fallback model: 65,537 deterministic samples; 37,044 voxel-deduplicated witnesses individually rechecked by MoveIt FK/FCL with zero collision and zero FK discrepancy.
- Before the restart, formal keyboard and synthetic-gamepad open/close pairs each ran for 1 s per direction, with measured aperture 34.5–41.2%. All 70,494 retained original SDK frames have no sequence gaps, no alarms and command ID 1; no ServoJ or arm movement command was submitted. Geometry raw values matched actual CAN aperture in all observed cycles.
- That session ended through the user's device restart, not a clean release. Its two incomplete final JSONL lines, incomplete final input and interrupted-session result remain preserved in `before-reboot/` and `test-exit.json`.
- The first post-reboot session restored EnableRobot(0.750,0,-14,46,0) once and confirmed payload feedback and a new stable window. Enable transient motion is preserved; no zero-displacement claim is made. Its 37,611 SDK frames have no gaps or alarm, command ID 1 throughout. This session exited cleanly before the fixture update.
- The lower-page diagnostics caused the browser to scroll the real video out of view, triggering `preview_stale`. The original stop behavior remains. Test-only diagnostics now float when expanded and reveal the shared observation row; normal production sessions do not expose these controls. Failed attempts are retained.
- Final corrected test session `20261016-221039-2095abf6`: 5 keyboard input intervals and 2 synthetic-gamepad intervals through the formal page; lower limit, reversal, upper limit, continued input at the upper limit, release and input-source switching passed. No gripper fault or unfinished input; clean exit and CPU governor restoration confirmed. 32,341 original SDK frames have no sequence gaps or alarms, command ID 1 throughout, no ServoJ/Enable/ClearError in this session. Joint peak-to-peak maximum was 0.061112° while the arm held position; no zero-displacement claim.
- All 3,885 CAN transmissions were raw 39–153 and all changed batches were 4–7 counts. Actual readback spanned raw 38–153 (14.90–60%); the lower-end one-count difference is retained and is within the existing arrival tolerance. Geometry matched actual raw aperture and all eight table angles in every valid cycle. 1,858 cycles differed from the sent target, confirming the display did not substitute the command for feedback. Screenshots `formal-fixed-low.png` / `formal-fixed-high.png` retain the same view for comparison.
- 4,848 unique observed geometry checks: callback P50/P95/P99/max 2.828/16.208/22.766/29.586 ms, no sweep-budget exhaustion. One expired startup result was denied while uninitialized and unprepared; this is not hidden as a collision. These are observed solver callback timings, not input-to-motion latency or a full arm/gripper performance qualification.
- The HTTP observer ended with 503 when the owned test session was deliberately shut down; its already-written samples remain. Complete native audit is authoritative for the result above.
- Rollback model/binary verification passed. Final production session and capture are recorded below after normal release.
- First production startup `20261016-221725-de1cbf46` failed on its first RGB frame: the existing UVC timestamp reader raised `Truncated Linux UVC metadata block`. RGB/depth delivered zero frames, so readiness never passed. The camera exited 1, gateway exited 0, CPU governor restored and no video-device owner remained. Its `clean=false` result, raw audit and original stack are retained in `camera-start-failure/`. No camera guard or timestamp parser was relaxed; the next fresh owned startup recovered as recorded below. This is separate from the passed gripper tests.

## Published session

`20261016-222135-2973f1c1` runs the same verified model/binary and corrected encoder adapter, with synthetic inputs disabled. Entry: `output/dynamic_gripper_operator_269f.py operator`; normal exit file: `output/dynamic_gripper_operator_269f_operator.stop`. All deployed platform source hashes match the saved final source.

The original camera configuration started successfully on the fresh owned attempt. The first-frame metadata failure above is retained and is not claimed permanently fixed. Gripper initialization to 35% succeeded through the formal page. A 10 s / 91-snapshot observation confirmed unprepared, stationary, enabled, payload matched, no error/collision, command ID 1, recording idle, current RGB and current gripper geometry. Gripper readback was approximately 34.5–34.9%. No repeated EnableRobot, ClearError or arm movement was sent during publication. `published-dashboard.png` shows the resulting normal page; the failed-start browser tab may remain separate.

Use Prepare Control before operating. Keyboard Space + F/R or the normal mapped gamepad controls exercise the same path tested above. Collision bounds shows rigid components in blue and moving fingers in yellow. No physical simulator, target clicking or additional control connection was added.

This result verifies dynamic geometry and bounded empty-gripper operation. Individual linkage angles remain nominal, and it does not certify object-contact deformation, external obstacles/cables, all arm poses/speeds, or the paused full-platform Goal.

## Reuse and reproduction

The official OmniPicker CAD links, existing four-bar closure implementation and pinned MoveIt 2.12.4 / FCL remain authoritative inputs. Rendering still uses the complete urdf-loader 0.13.1 URDFViewer and Three.js 0.186.1. No new visualization library or replacement solver is introduced.

Relevant sources: `articulated_gripper_assets.py`, `dynamic_gripper_fallback.py`, `verify_dynamic_gripper_geometry.py`, `gripper_probe.cpp`, `dynamic_gripper.hpp`, `qualify_dynamic_gripper.py`, `gripper_geometry.py`, `WorkspaceView.tsx`, `workspaceRenderer.ts`. The full CAD builder supports `--dynamic-gripper`; formal frontend sources are copied by the existing reproducible upstream preparation workflow.

## Rollback

`output/dynamic-gripper-v269/baseline` retains v268 assets, served dashboard, workstation configuration, previous solver pin/binary and actual deployed Python sources. After unprepared, stationary, recording idle and normal owned-session exit:

```powershell
teleop-platform/.venv-launcher/Scripts/python.exe teleop-platform/scripts/switch_dynamic_gripper_v269.py --rollback --check
teleop-platform/.venv-launcher/Scripts/python.exe teleop-platform/scripts/switch_dynamic_gripper_v269.py --rollback --password
```

The switch does not enable, initialize, clear alarms or move devices. Use the existing launcher afterward. Models and failed test artifacts are preserved.
