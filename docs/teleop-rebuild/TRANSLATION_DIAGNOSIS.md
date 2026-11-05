# Translation investigation — v244, 2026-10-11

Scope: read-only diagnosis of the operator's actual keyboard operation in session
`20261011-220215-9b859051`, approximately 22:53–22:57. No new motion, Prepare,
Enable, CAN initialization, parameter changes, deployment or session restart.
The full Goal remains paused. This is a diagnosis, not a completed smoothness fix.

## Current limits

- Grasp-center TCP: nominal fixed 150.4 mm offset, `omnipicker-r2`.
- Translation: 50 mm spherical radius about the session's original preparation
  position. This is not ±50 mm independently on each axis. Re-preparation retains
  the original center.
- Relative rotation/joint caps are disabled. Absolute official-model joint limits
  retain a 5° margin at each end.
- Normal input limits: 30 mm/s, 10°/s before the user's multiplier and solver limits.
- Current boundary damper starts reducing radial outward speed according to
  `3 * max(0, radius - distance - reserve)`; reserve is 0.5 mm. At 30 mm/s pure
  radial motion it begins 10.5 mm before the boundary. Inward motion is unaffected.
- Fresh read-only snapshot: unprepared, `page_blur`, stationary, recording idle;
  TCP about 43.24 mm from the session origin. These are sampled facts, not a promise
  of future device state.

## Findings from the recorded operation

1. **Boundary handling changes the requested direction.**
   `workspace_velocity.damp_translation` subtracts a radial component rather than
   scaling the original vector. Away from a principal radius, a single-axis input
   therefore gains other-axis components and slides along the sphere. Of 1,917
   sent translation targets, 356 contain this adjustment. Offline replay of all
   356 recorded boundary inputs through the deployed function reproduces the
   recorded effective velocities with maximum error 1.91e-17 m/s.

   One actual example is a base-frame `[0, -12, 0]` mm/s input becoming approximately
   `[+2.395, -0.809, +0.039]` mm/s. This is a software-generated command change,
   not a measured claim about exact instantaneous hardware velocity. The current
   behavior can explain the operator's unintended sideways-motion sensation.

2. **The trajectory is often scaled near a singularity.**
   Translation target statuses: 759 normal, 643 approaching-singularity slowdown,
   515 leaving-singularity slowdown. These are official MoveIt status codes 0,
   1 and 3, respectively. They are not proof of a collision or an E6 hardware fault.
   The different scaling explains why identical input can have different speed
   in different poses/directions; it does not by itself prove the cause of all jerks.

3. **The send cadence has real jitter.**
   Consecutive sends within each continuous translation epoch:

   | X5-local metric | Median | P95 | Maximum |
   |---|---:|---:|---:|
   | ServoJ send interval | 50.044 ms | 67.790 ms | 99.534 ms |
   | Solver request-to-result | 3.993 ms | 4.624 ms | 21.794 ms |
   | Computed result to send | 13.758 ms | 29.237 ms | 65.929 ms |

   Minimum send interval is 21.138 ms. Three translation candidate parent IDs were
   not themselves sent, producing skipped intermediate steps. This warrants
   investigation of local result delivery/consumption and sending. The evidence
   does not establish which scheduling/IPC/acknowledgment stage caused each gap,
   nor does it establish an input-to-physical-motion latency or a Wi-Fi diagnosis.

4. **Repeated arbitrary Stops were not found in this operation.**
   All 33 recorded Stop requests follow a zero-axis input with deadman still true.
   There are 33 motion epochs and 2,600 submitted/confirmed motion events total
   (1,917 translation, 683 rotation). No workspace-blocked event was recorded.
   Stop acknowledgments taking roughly 0.64–0.70 s concern stopping/restarting;
   that duration must not be presented as every motion-command's latency.

## Next focused changes

First replace radial projection with direction-preserving speed reduction so an
X/Y/Z command cannot acquire an unrequested orthogonal component. Retain the same
workspace and independent hard target check; test oblique boundary approaches,
base/tool frames, inward retreat, tangency, and numerical boundary cases.

Separately reproduce and correct the local result-to-send timing jitter. Keep
the original source deadlines, unique-target semantics and release handling.
Do not expand the workspace, remove singularity handling, replay old targets, or
increase deadlines to make the symptom disappear. Do not describe an offline
change as a hardware smoothness qualification.

## Evidence

`output/translation-diagnosis-v244/`: fresh `initial-state.json`, downloaded native
`control.log`, bounded session `audit-extract.json`, `analysis.json`,
`verified-summary.json`, and read-only extraction/analysis/replay scripts. Original
X5 audit files remain unchanged. The extraction includes a fixed-size snapshot
of the append-only audit, not a claim that the still-running session has ended.
