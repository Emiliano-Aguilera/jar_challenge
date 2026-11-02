# Temporary decoy-entry experiment

This draft branch is an execution fixture, not a production navigation change.
Its pruebas.launch.py starts real AMCL/Nav2 and the controlled client, with no
mission, perception or identity TF. Competition launch and runtime parameters
are unchanged. The source client is copied from the uncommitted access-audit
work for this isolated experiment; that original worktree is untouched.

Start the normal simulation session with pruebas.launch.py and ejemplo_senuelos.
No goal is sent until /jar/diagnostico/iniciar (std_srvs/SetBool): false selects the
single exact first goal; true selects the aligned approach and the same final
goal. Query /jar/diagnostico/estado (std_srvs/Trigger) for JSON progress/results.
Each simulator instance accepts one trial and permanently inhibits output at its
end. Restart the session before another trial. All coordinates live in the
external test fixture casos.json, never production parameters.

The client checks effective footprint and consumers before dispatch, records real
poses and action results, and aborts later segments on error. Intermediate goals
still use the normal Nav2 goal checker: requested alignment is not guaranteed;
inspect actual arrival poses. Ground-truth samples support offline clearance
checks but do not themselves certify continuous collision-free motion.

Validation: 24 existing audit/transport tests pass; clean offline build installs
the driver/fixture; parameter and static checks pass. Actual movement pending.
