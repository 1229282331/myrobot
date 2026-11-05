# Request-state Servo candidate v342

Experimental source; **not deployed or qualified for physical motion**.
The production runtime pin remains unchanged.

`source/` retains the upstream BSD notices and all reviewed geometry code.
`source-manifest.json` binds every source byte; `baseline-manifest.json` binds
the v337 parent. `request-state.patch` contains the three changed source files.
The request's measured joint state supplies both model-frame transforms and
collision checking; target seeds remain separate. The first monitor delivery
still gates initialization. Feedback, callback and motion limits are unchanged.

`tests/probe.cpp` compares the complete Servo result against the saved parent,
including a deliberately stale monitor and dirty FK cache. PC qualification
uses no physical IO and disables smoothing to isolate frame semantics.
The full smoothing/adapter RDK ABBA and loaded-chain qualification remain
required before deployment. Reproduction tooling and raw failed attempts are
preserved under `output/*request*342*` and `evidence/request-state-v342`.
See `docs/teleop-rebuild/REQUEST_STATE_V342.md`.
