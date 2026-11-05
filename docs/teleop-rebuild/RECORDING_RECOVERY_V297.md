# v297 — recording recovery and background transfer

Released 2026-10-18. User has taken over teleoperation; no autonomous cup grasp or new physical motion test was performed in this repair.

## Observed failure

The user's Gamepad episode `20261018T115459Z_e8856793` ended unexpectedly during 0.8x rotation. Input was fresh: the last four conservative age upper bounds were 11.54–19.20 ms. The last consumed solver candidate preceded cancellation by **154.139996 ms**. One `solver_deadline` rejection occurred. The original gateway treated a candidate gap over 150 ms as a full suspension, cancelled Prepare, aborted recording, then prohibited preparation throughout transfer. That turned one interruption into several minutes of unavailable control.

The failure reply was overwritten by a subsequent reset result in the original latest-state diagnostic channel. The capture does **not** distinguish the underlying C++ computation, IPC scheduling, and feedback-deadline contribution. Condition number near 33 can explain some real singularity scaling, but does not explain the recorder/Prepare lifecycle failure. No claim is made that every latency spike has been eliminated.

Read-only review: 996 cycle samples, 6,337 original SDK frames without sequence gaps or reported alarm/collision, 732 submitted arm commands with 732 accepted acknowledgements. The retained result remains `aborted`; PC and remote verification completed. The archive is 253,971,426 bytes, SHA-256 `f4a304f560b4dd9a09a27740ce805dc35d849f8b9adc63be2fd1994068fe1ab3`. Twenty invalid action associations remain reported. Verification is not successful task completion or replay certification.

![Original failure timing](../../evidence/solver-stream-v297/manual-analysis-delivery/failure-timing.png)

## Changes

- A solver-stream gap still requests Stop at the original 150 ms threshold, clears old targets, and pauses input. It preserves Prepare and the running recording. Held input and stale neutral packets cannot resume movement: a fresh neutral packet using a post-pause echo is required before a new command.
- Actual feedback failure, model failure, uncertain/rejected Stop, explicit Stop, page disconnect and existing hardware fault paths still cancel Prepare and terminate an active recording.
- Recording start and sealing still impose their original quiet-period constraints. After sealing, `transfer_pending` no longer blocks explicit Prepare or repeatedly suspends prepared control. A transfer checksum failure does not inhibit control or rewrite a sealed episode. A receipt never prepares control.
- Candidate rejection details retain the original identity, epoch, input/feedback timing, deadline, callback/roundtrip and geometry result across later neutral/success replies. Rejections are logged through the existing asynchronous human-readable log channel.
- Frontend status describes release/centering recovery correctly. Transfer progress states that control is available and the next recording waits for verification. Formal extension sources are copied through the existing reproducible 13-patch upstream preparation process.

The next recording still waits for the current transfer to verify. Concurrent transfer plus a new native RGB-D recording has not been qualified here. File volume and transfer duration are not claimed to have improved. Explicit Stop continues to mark an active recording aborted; normal endings use Success or Failure.

Original 100 ms feedback/candidate validity, 250 ms input freshness, 37/42/50 ms geometry/callback/cycle limits, model/ACM, TCP 151.4 mm, payload 0.750 kg / COM (0, −14, 46) mm, CAN Force 255, speed and smoothing are unchanged. Physical replay remains unqualified (`physical_replay_allowed=false`). The earlier all-zero depth-frame issue is not resolved by this change.

## Validation

| Check | Evidence / result |
| --- | --- |
| Old lifecycle reproduced | 4 transfer/Prepare tests failed against the old gating; one new maintenance-method test was absent, recorded separately. |
| Final complete backend suite | 1,038 passed, 8 skipped; `evidence/solver-stream-v297/backend-final-tests.txt`. |
| Final frontend suite | 203 passed; `frontend-final-tests-b.txt`. The preceding failure was an outdated test expectation about an explicitly requested reconnect; retained. |
| Production build | TypeScript and Vite passed; `frontend-final-build.txt`. |
| Real journal fixtures | Success/failure/aborted sealing, control during pending transfer, rejected overlapping recording, failed checksum, explicit Stop and immutable sealed files. No physical hardware transport. |
| X5 isolated worker comparison | Original binary/model, loopback-only namespace, synthetic feedback with 100 ms plant delay, 0.8x rotation, dual audit journals. Four ABBA runs, 1,196 candidates consumed, no active deadline rejects. Original single-frame delay was not reproduced. Two normal release rejections retained. |
| X5 qualification scope | Candidate A worker/intent sources are identical to final B. Final B gateway lifecycle is covered by PC tests; the worker ABBA test does not run the gateway. Exact source hashes and scope are in `qualification-final.json`. |
| Online read-only verification | New owner, 20 fresh states and a separate 500-frame original SDK window, no frame gaps/alarm/collision, command ID 19216 constant. Unprepared, stationary, Mode5 / Enable1, correct load/COM, no gripper target sent. |
| UI | Formal Gamepad page reloaded; real RGB and live 3D visible. Browser currently needs physical gamepad activation; no synthetic input enabled. |

Initial missing-PYTHONPATH suite failures and initial isolated missing-assets failure remain archived. The startup 503 before observation readiness recovered. Read-only verification helper corrections (environment path, HTML newline normalization, UTF-8 and local variable reuse) did not change production code or device state.

## Deployment and rollback

Previous owner `20261018-185800-6c64f055` exited cleanly: camera/gateway 0, CPU and Wi-Fi policies restored. Local media exit 1 with empty cleanup errors remains recorded.

Current owner: `20261018-201806-fb8b3437`, launcher `output/recording_operator_297.py operator`, unified exec 75959. Normal stop file: `output/recording_operator_297_operator.stop` (not created at delivery). No Enable, ClearError, parameter write, Prepare or commanded physical motion was sent by the assistant this turn.

The actual prior deployment is preserved under `output/solver-stream-v297/baseline`; candidate A and its exact qualification are preserved in `candidate-a`. Final release and checksums are in `release` / `release.json`. `baseline-supplement.json` binds the two additional frontend baseline files copied before upstream preparation.

Rollback: normally release the owner while unprepared, stationary and recording ended; run `teleop-platform/scripts/switch_solver_stream_v297.py --rollback --check`, then the same command with `--rollback` and the established credential mechanism. The script verifies no local service listeners, a clean previous session, the remote owner lock, unchanged configuration/binary pin, source/build hashes and final validation. It does not start an owner or command the robot.

Binary: `7e7d30635e011c26ba8aed2d94ec02876986054ae0cf25180004273e8146e6fa`; assembly: `3b4003e47dc241273476c7a01a7102963d06ed59767ddd010afdaeeba72a72d7`. Goal remains paused. No Git commit made.

![Restored formal platform](../../evidence/solver-stream-v297/final-platform.png)
