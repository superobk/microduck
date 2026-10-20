#!/bin/bash
# Run manually as root on the supported, torque-released robot. No init/enable.
set -euo pipefail
cd "$(dirname "$0")"
[[ $(id -u) == 0 ]] || { echo 'Run as root' >&2; exit 1; }
sha256sum -c SHA256SUMS
CANDIDATE=/opt/robot/local/imu-acceptance-63e0fa0c
TRACE=/var/lib/robot/diagnostics/imu-acceptance-trace
REVIEW=/var/lib/robot/reviews/imu-acceptance-63e0fa0c
DROPIN=/etc/systemd/system/robotd.service.d/99-imu-acceptance.conf
if [[ -e "$DROPIN" ]]; then
  cmp 99-imu-acceptance.conf "$DROPIN"
  PID=$(systemctl show robotd -p MainPID --value)
  [[ $(readlink -f "/proc/$PID/exe") == "$CANDIDATE/robotd" ]]
  printf '%s  %s\n' d19c9ccf586c27a0b1581cab35fcad57fb9c7746e014953fb38429201d919a39 "/proc/$PID/exe" | sha256sum -c -
  echo 'Existing acceptance candidate verified.'
  exit 0
fi
[[ ! -e "$CANDIDATE" && ! -e "$DROPIN" && ! -e "$REVIEW" ]] || {
  echo 'Candidate or review already exists; inspect previous installation first.' >&2; exit 1;
}
systemctl is-active --quiet robotd
[[ $(systemctl show robotd -p ConditionResult --value) == yes ]]
ROBOT_PID=$(systemctl show robotd -p MainPID --value)
# This package preserves the known --socket-only service. Refuse to discard custom flags.
python3 - "$ROBOT_PID" <<'CHECK'
import pathlib,sys
args=pathlib.Path('/proc/'+sys.argv[1]+'/cmdline').read_bytes().split(b'\0')
assert args[1:] == [b'--socket', b'/run/robotd.sock', b''], 'Custom robotd flags: preserve them before deployment'
CHECK
OLD_EXE=$(readlink -f "/proc/$ROBOT_PID/exe")
[[ -f "$OLD_EXE" ]] || { echo 'Original executable unavailable; cannot guarantee rollback.' >&2; exit 1; }
systemctl stop padd btd mediad
# Require a new successful torque-off log; an accepted IPC request alone is insufficient.
CURSOR=$(journalctl -u robotd -n 0 --show-cursor --no-pager | sed -n 's/^-- cursor: //p')
[[ -n "$CURSOR" ]]
robotctl robot relax --yes --json
TORQUE_OFF=no
for _ in {1..15}; do
  if journalctl -u robotd --after-cursor="$CURSOR" --no-pager | grep -Fq 'robot.relax: torque off'; then
    TORQUE_OFF=yes
    break
  fi
  sleep 1
done
[[ "$TORQUE_OFF" == yes ]] || { echo 'No fresh torque-off confirmation; robotd was not replaced.' >&2; exit 1; }
install -d -m 700 "$REVIEW" "$TRACE"
printf '%s\n' "$OLD_EXE" > "$REVIEW/old-executable.txt"
sha256sum "$OLD_EXE" > "$REVIEW/old-executable.sha256"
systemctl cat robotd > "$REVIEW/robotd-unit-before.txt"
cp -a /etc/robot/robotd.toml "$REVIEW/robotd.toml.before"
sha256sum /etc/robot/robotd.toml > "$REVIEW/config.sha256"
readlink -f /opt/robot/policies/current > "$REVIEW/policy-target.txt"
find -L /opt/robot/policies/current -type f -name '*.onnx' -exec sha256sum {} + > "$REVIEW/models.sha256"
robotctl policy list --json > "$REVIEW/policies-before.json"
SWITCHED=yes
COMPLETE=no
restore_on_error() {
  code=$?
  if [[ "$COMPLETE" != yes && "$SWITCHED" == yes ]]; then
    echo '安装校验失败，移除本包覆盖并恢复原服务；保持躯干有支撑。' >&2
    systemctl stop robotd || true
    rm -f -- "$DROPIN"
    systemctl daemon-reload
    systemctl start robotd || true
  fi
  exit "$code"
}
trap restore_on_error EXIT
systemctl stop robotd
install -d -m 755 "$CANDIDATE" /etc/systemd/system/robotd.service.d
install -m 755 robotd-linux-aarch64 "$CANDIDATE/robotd"
install -m 644 99-imu-acceptance.conf "$DROPIN"
systemctl daemon-reload
systemctl start robotd
sleep 3
systemctl is-active --quiet robotd
ROBOT_PID=$(systemctl show robotd -p MainPID --value)
[[ $(readlink -f "/proc/$ROBOT_PID/exe") == "$CANDIDATE/robotd" ]]
printf '%s  %s\n' d19c9ccf586c27a0b1581cab35fcad57fb9c7746e014953fb38429201d919a39 "$CANDIDATE/robotd" | sha256sum -c -
sha256sum -c "$REVIEW/config.sha256"
sha256sum -c "$REVIEW/models.sha256"
[[ $(readlink -f /opt/robot/policies/current) == $(cat "$REVIEW/policy-target.txt") ]]
robotctl version --json | tee "$REVIEW/version-after.json"
robotctl policy list --json | tee "$REVIEW/policies-after.json"
INVOCATION=$(systemctl show robotd -p InvocationID --value)
journalctl "_SYSTEMD_INVOCATION_ID=$INVOCATION" --no-pager > "$REVIEW/startup-after.log"
grep -F 'mode=Predict' "$REVIEW/startup-after.log"
grep -F 'IMU raw capture opened' "$REVIEW/startup-after.log"
grep -F 'policy loaded' "$REVIEW/startup-after.log"
robotctl robot relax --yes --json
printf 'Installed supported-bench-only predict/40ms candidate. Check policies-before/after in %s. No init/enable was sent. padd remains stopped.\n' "$REVIEW"

COMPLETE=yes
