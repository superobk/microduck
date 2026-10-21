#!/bin/bash
# Support the robot before running. If IPC is unavailable, keep it supported throughout.
set -euo pipefail
[[ $(id -u) == 0 ]]
REVIEW=/var/lib/robot/reviews/imu-acceptance-63e0fa0c
DROPIN=/etc/systemd/system/robotd.service.d/99-imu-acceptance.conf
grep -Fxq 'ExecStart=/opt/robot/local/imu-acceptance-63e0fa0c/robotd --socket /run/robotd.sock' "$DROPIN"
sha256sum -c "$REVIEW/old-executable.sha256"
robotctl robot relax --yes --json || echo 'IPC unavailable: keep robot physically supported during rollback.' >&2
systemctl stop padd
systemctl stop robotd
rm -- "$DROPIN"
systemctl daemon-reload
systemctl start robotd
sleep 3
systemctl is-active --quiet robotd
ROBOT_PID=$(systemctl show robotd -p MainPID --value)
[[ $(readlink -f "/proc/$ROBOT_PID/exe") == $(cat "$REVIEW/old-executable.txt") ]]
sha256sum -c "$REVIEW/old-executable.sha256"
robotctl robot relax --yes --json
robotctl version --json
printf 'Original executable restored. padd remains stopped; no init/enable was sent.\n'
