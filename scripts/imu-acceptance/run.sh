#!/bin/bash
# User-run Mac front door. No connection for --check-local. No motion during installation.
set -euo pipefail
umask 077
STAGE=bench
LONG=()
ACTION=run
TARGET="${ROBOT_TARGET:-root@172.19.8.218}"
PACKAGE="${IMU_ACCEPT_PACKAGE:?Set IMU_ACCEPT_PACKAGE to the prepared local package}"
OUTPUT="${IMU_ACCEPT_OUTPUT:?Set IMU_ACCEPT_OUTPUT to the local log directory}"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --stage) STAGE="${2:?bench|stand|walk}"; shift 2 ;;
    --long) LONG=(--long); shift ;;
    --check-local) ACTION=check; shift ;;
    --fetch-last) ACTION=fetch; shift ;;
    --rollback) ACTION=rollback; shift ;;
    root@*) TARGET="$1"; shift ;;
    *) echo 'Usage: --stage bench|stand|walk [--long] [root@host] | --check-local | --fetch-last | --rollback'; exit 2 ;;
  esac
done
[[ "$TARGET" =~ ^root@[a-zA-Z0-9.-]+$ ]] || exit 2
[[ "$STAGE" =~ ^(bench|stand|walk)$ ]] || exit 2
REMOTE=/var/tmp/imu-acceptance-63e0fa0c
EXPECTED=d19c9ccf586c27a0b1581cab35fcad57fb9c7746e014953fb38429201d919a39
(cd "$PACKAGE" && shasum -a 256 -c SHA256SUMS)
if [[ "$ACTION" == check ]]; then
  echo '本地校验完成，没有连接机器人。'
  exit 0
fi
mkdir -p "$OUTPUT"
fetch() {
  local name="$1"
  [[ "$name" =~ ^(bench|stand|walk)-[0-9TZ-]+$ ]] || return 2
  mkdir -p "$OUTPUT/$name"
  # Finished marker only appears once collector and file hashes have been finalized.
  ssh -o ConnectTimeout=8 "$TARGET" "test -f /var/lib/robot/diagnostics/imu-acceptance/$name/finished-acceptance.json && tar -czf /var/tmp/$name.tar.gz -C /var/lib/robot/diagnostics/imu-acceptance $name && cd /var/tmp && sha256sum $name.tar.gz > $name.tar.gz.sha256"
  scp "$TARGET:/var/tmp/$name.tar.gz" "$TARGET:/var/tmp/$name.tar.gz.sha256" "$OUTPUT/$name/"
  (cd "$OUTPUT/$name" && shasum -a 256 -c "$name.tar.gz.sha256" && tar -xzf "$name.tar.gz" && cd "$name" && shasum -a 256 -c SHA256SUMS)
  echo "日志已取回：$OUTPUT/$name/$name"
  cat "$OUTPUT/$name/$name/result.json"
}
if [[ "$ACTION" == fetch ]]; then
  [[ "$(sed -n '1p' "$OUTPUT/last-session.txt")" == "$TARGET" ]] || { echo '目标与上次采集不同'; exit 1; }
  fetch "$(sed -n '2p' "$OUTPUT/last-session.txt")"
  exit
fi
echo '先固定/托稳躯干，腿脚无阻挡。停止其他控制页面。'
echo 'bench：安装/复用保护候选，仅relax采集；stand/walk：确认提示后将init/enable。'
echo '本轮会停止padd/btd/mediad；结束后保持停止。成功结束不自动rollback。'
read -r -p '已准备好请输入 YES：' answer
[[ "$answer" == YES ]] || exit 1
if [[ "$ACTION" == rollback ]]; then
  ssh -tt "$TARGET" "cd $REMOTE && sha256sum -c SHA256SUMS && bash rollback-board.sh"
  exit
fi
if [[ "$STAGE" == bench ]]; then
  ssh -o ConnectTimeout=8 "$TARGET" "install -d -m 700 $REMOTE"
  scp "$PACKAGE/"* "$TARGET:$REMOTE/"
  ssh -tt "$TARGET" "cd $REMOTE && bash install-board.sh"
else
  ssh "$TARGET" "cd $REMOTE && sha256sum -c SHA256SUMS"
fi
NAME="$STAGE-$(date -u +%Y%m%dT%H%M%SZ)-$$"
printf '%s\n%s\n' "$TARGET" "$NAME" > "$OUTPUT/last-session.txt"
CODE=0
ssh -tt -o ServerAliveInterval=5 -o ServerAliveCountMax=2 "$TARGET" \
  "cd $REMOTE && python3 accept.py --stage $STAGE --run $NAME --sha256 $EXPECTED ${LONG[*]:-}" || CODE=$?
fetch "$NAME" || { echo '尚未取回。保持物理支撑；连接恢复后运行同一命令加 --fetch-last。'; exit 1; }
exit "$CODE"
