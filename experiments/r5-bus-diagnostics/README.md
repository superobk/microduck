# R5 bus diagnostic experiment (CPU-only reference)

This experiment deliberately does not change robotd, protocol, firmware or servo commands. CPU regression tests are implemented as a standalone Python reference model; see the downloadable experiment bundle in the accompanying discussion. This branch is **not yet** a deployed 0.15.0 bus-driver patch.

Acceptance: normal reads; named single-device failure; consecutive failures; recovery clears current run without clearing cumulative counters; unnamed failure; cumulative short-lived drops. Device IDs must originate in the actual driver error; do not infer them from a timeout.

Hardware safety: retain 1 Mbps, IMU ID 200, ordinary Sync Read 0x82 and `fast_sync_read = false`. Stop policy and all competing bus writers before attaching any independent reader. Do not send goal-position, torque-enable or control commands. Require an explicit operator check of torque-disabled state. Record monotonic timestamps, ID, per-ID failures, unnamed failures, IMU WHO/GSEQ/QSEQ/SPIERR/FLAGS, measured Hz and supply voltages. CPU-only tests cannot validate physical failure causes.

Follow-up work: review the exact daemon-v0.15.0 driver and rustypot API before porting error attribution; keep the original IPC schema intact.