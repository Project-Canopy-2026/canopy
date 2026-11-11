#!/usr/bin/env bash
# pretest.sh - set up and check the robot, free the Jetson's CPU, then start
# the Foxglove operator stack.
#
# Run it over SSH from your laptop, inside tmux. Stopping the desktop kills any
# terminal that belongs to it, and tmux keeps the stack running if Wi-Fi drops:
#
#   ssh teamj@<jetson>
#   tmux new -s test            # reattach later with: tmux attach -t test
#   ~/dev/ros2_ws/src/canopy/scripts/pretest.sh
#
# Steps (the setup commands are the ones in README.md):
#   1. Network + CAN setup
#   2. Check the W200, GNSS, RTK serial, xArm, Velodyne, Arduino and the LINAK
#      CAN bus. Stops here if anything failed, before the desktop is touched.
#   3. Stop the desktop and background services, close leftover apps
#   4. Report CPU and RAM use
#   5. ros2 launch launch/foxglove_operator.launch.py
#
# Options:
#   --check-only   steps 1-2 only (safe to run from the desktop)
#   --keep-ui      skip step 3
#   --force        go on to steps 3-5 even if a check failed
#
# After the test: scripts/posttest.sh brings the desktop back.

set -o pipefail

REPO="$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)"   # .../src/canopy
WS="$(cd "$REPO/../.." && pwd)"                              # .../ros2_ws

# ── Hardware (same values as README.md and the launch/config files) ─────────
W200_IF=enx0050b6e57539      # USB Ethernet to the W200
SENSOR_IF=enP8p1s0           # Ethernet to GNSS, xArm and Velodyne
W200_IP=192.168.131.1
GNSS_IP=192.168.0.222
XARM_IP=192.168.1.201
LIDAR_IP=192.168.1.202
LIDAR_PORT=2368              # Velodyne data packets (UDP)
CAN_IF=can1                  # Kvaser, LINAK actuators (planting_bringup default)
CAN_BITRATE=125000
CAN_RESTART_MS=100
ARDUINO=/dev/arduino         # udev symlink, see planting_bringup.launch.py
RTK_SERIAL=/dev/ttyUSB0      # FTDI to the GNSS, RTK corrections (ntrip_client.yaml)
NTRIP_HOST=caster.emlid.com
NTRIP_PORT=2101
LAUNCH_FILE="$REPO/launch/foxglove_operator.launch.py"

CHECK_ONLY=false
KEEP_UI=false
FORCE=false
for arg in "$@"; do
  case "$arg" in
    --check-only) CHECK_ONLY=true ;;
    --keep-ui)    KEEP_UI=true ;;
    --force)      FORCE=true ;;
    -h|--help)    sed -n '2,25p' "$0"; exit 0 ;;
    *) echo "Unknown option: $arg (see --help)"; exit 2 ;;
  esac
done

if [ -t 1 ]; then
  G=$'\e[32m'; R=$'\e[31m'; Y=$'\e[33m'; B=$'\e[1m'; N=$'\e[0m'
else
  G=; R=; Y=; B=; N=
fi
header() { echo; echo "${B}== $* ==${N}"; }
die()    { echo "${R}ERROR:${N} $*"; exit 1; }
ask()    { local a; read -r -p "$1 [y/N] " a; [[ "$a" =~ ^[Yy] ]]; }

# Show each setup command like the README does, and note failures.
SETUP_ERRORS=0
run() {
  echo "  \$ $*"
  if ! "$@"; then
    echo "    ${R}failed${N}"
    SETUP_ERRORS=$((SETUP_ERRORS + 1))
    return 1
  fi
}

RESULTS=()
FAILS=0
WARNS=0
pass() { RESULTS+=("${G}PASS${N}  $(printf '%-10s' "$1") $2"); }
fail() { RESULTS+=("${R}FAIL${N}  $(printf '%-10s' "$1") $2"); FAILS=$((FAILS + 1)); }
warn() { RESULTS+=("${Y}WARN${N}  $(printf '%-10s' "$1") $2"); WARNS=$((WARNS + 1)); }
pingable() { ping -c 2 -W 1 -q "$1" >/dev/null 2>&1; }

# ── 0. Preflight ─────────────────────────────────────────────────────────────
[ "$EUID" -eq 0 ] && die "Run as your normal user, not with sudo. The script calls sudo itself."

if ! $CHECK_ONLY && ! $KEEP_UI; then
  if [ -z "${SSH_CONNECTION:-}" ] || [ "${TERM_PROGRAM:-}" = vscode ]; then
    die "Step 3 stops the desktop, which would kill this terminal.
       Run this over plain SSH from your laptop (not a VS Code terminal),
       or use --check-only / --keep-ui."
  fi
  if [ -z "${TMUX:-}" ]; then
    echo "${Y}Not inside tmux:${N} if SSH drops, the robot stack started at the end stops with it."
    ask "Continue anyway?" || { echo "Start tmux with:  tmux new -s test"; exit 1; }
  fi
fi

sudo -v || die "sudo is needed for the network and CAN setup."

# Leftover ROS processes would fight the new stack (second foxglove_bridge on
# the same port, a second CANopen master on the LINAKs, ...).
mapfile -t LEFTOVER < <(pgrep -u "$USER" -af 'ros2 launch|ros2 run|ros2_ws/install/|/opt/ros/humble/lib/' \
                        | grep -v '_ros2_daemon')
if [ "${#LEFTOVER[@]}" -gt 0 ]; then
  header "ROS processes already running"
  printf '  %s\n' "${LEFTOVER[@]}" | cut -c1-150
  if ask "Stop them before continuing?"; then
    pids=$(printf '%s\n' "${LEFTOVER[@]}" | awk '{print $1}')
    kill -INT $pids 2>/dev/null
    for _ in $(seq 15); do
      alive=$(for p in $pids; do kill -0 "$p" 2>/dev/null && echo "$p"; done)
      [ -z "$alive" ] && break
      sleep 1
    done
    [ -n "$alive" ] && kill -KILL $alive 2>/dev/null
    echo "  stopped."
  else
    die "Stop them first (Ctrl-C in their terminals)."
  fi
fi

# ── 1. Network + CAN setup (README.md) ───────────────────────────────────────
header "1. Network and CAN setup"

echo "W200 link:"
run sudo ip addr flush dev "$SENSOR_IF"
run sudo ip link set "$W200_IF" up
run sudo ip addr replace 192.168.131.100/24 dev "$W200_IF"
# Keep the default route (internet, Tailscale) on Wi-Fi, not the W200.
while ip route show default | grep -q "via $W200_IP"; do
  run sudo ip route del default via "$W200_IP" || break
done

echo "GNSS, xArm, Velodyne link:"
run sudo ip addr add 192.168.0.10/24 dev "$SENSOR_IF"
run sudo ip addr add 192.168.1.100/24 dev "$SENSOR_IF"
run sudo ip link set "$SENSOR_IF" up

echo "CAN:"
run sudo ip link set "$CAN_IF" down
run sudo ip link set "$CAN_IF" up type can bitrate "$CAN_BITRATE" restart-ms "$CAN_RESTART_MS"
# can0 is the Jetson's built-in CAN. The stack does not use it; the README
# brings it up anyway, so do the same, but a failure here is not fatal.
if [ -e /sys/class/net/can0 ]; then
  echo "  \$ sudo ip link set can0 up type can bitrate $CAN_BITRATE"
  sudo ip link set can0 down 2>/dev/null
  sudo ip link set can0 up type can bitrate "$CAN_BITRATE" 2>/dev/null \
    || echo "    ${Y}can0 did not come up (not used by the stack)${N}"
fi

echo "ROS 2 environment:"
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$WS/install/setup.bash" || die "No $WS/install/setup.bash. Build the workspace first."
# Same as ~/.bashrc (a non-interactive script does not read it).
export ROS_DOMAIN_ID=1
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds_canopy.xml"
echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "  FASTRTPS_DEFAULT_PROFILES_FILE=$FASTRTPS_DEFAULT_PROFILES_FILE"
run ros2 daemon stop

# ── 2. Checks ────────────────────────────────────────────────────────────────
header "2. Checking sensors and CAN bus"

# DDS profile: ~/.bashrc uses a copy of the repo file; both must match.
if [ ! -f "$FASTRTPS_DEFAULT_PROFILES_FILE" ]; then
  fail "DDS" "$FASTRTPS_DEFAULT_PROFILES_FILE is missing (copy it from config/dds/)"
elif ! cmp -s "$FASTRTPS_DEFAULT_PROFILES_FILE" "$REPO/config/dds/fastdds_canopy.xml"; then
  warn "DDS" "~/.ros/fastdds_canopy.xml differs from config/dds/fastdds_canopy.xml"
else
  pass "DDS" "fastdds_canopy.xml matches the repo copy"
fi

# W200
echo "  W200 ..."
if [ ! -e "/sys/class/net/$W200_IF" ]; then
  fail "W200" "USB Ethernet adapter $W200_IF not found (unplugged?)"
elif [ "$(cat "/sys/class/net/$W200_IF/carrier" 2>/dev/null)" != 1 ]; then
  fail "W200" "no link on $W200_IF (cable unplugged or W200 off)"
elif ! pingable "$W200_IP"; then
  fail "W200" "$W200_IP does not answer ping"
else
  # README step 7. The bridge isn't running yet, so read it straight from the
  # W200's domain (0) with the bridge's DDS profile.
  bms=$(timeout 20 env ROS_DOMAIN_ID=0 \
          FASTRTPS_DEFAULT_PROFILES_FILE="$REPO/config/dds/fastdds_w200_bridge.xml" \
          ros2 topic echo --once --no-daemon /w200_0120/platform/bms/state \
          sensor_msgs/msg/BatteryState 2>/dev/null)
  pct=$(awk '/^percentage:/ {p=$2; if (p <= 1.0) p *= 100; printf "%.0f", p}' <<<"$bms")
  volt=$(awk '/^voltage:/ {printf "%.1f", $2}' <<<"$bms")
  if [ -z "$pct" ]; then
    warn "W200" "pings, but no battery message from /w200_0120/platform/bms/state in 20 s"
  elif [ "$pct" -lt 30 ]; then
    warn "W200" "pings, battery LOW: ${pct}% (${volt} V)"
  else
    pass "W200" "pings, battery ${pct}% (${volt} V)"
  fi
fi

echo "  GNSS ..."
if pingable "$GNSS_IP"; then pass "GNSS" "$GNSS_IP answers ping"
else fail "GNSS" "$GNSS_IP does not answer ping"; fi

echo "  RTK ..."
if [ ! -e "$RTK_SERIAL" ]; then
  fail "RTK" "$RTK_SERIAL missing (FTDI cable that sends corrections to the GNSS)"
elif ! timeout 5 bash -c "exec 3<>/dev/tcp/$NTRIP_HOST/$NTRIP_PORT" 2>/dev/null; then
  warn "RTK" "$RTK_SERIAL present, but can't reach $NTRIP_HOST:$NTRIP_PORT (no internet = no RTK fix)"
else
  pass "RTK" "$RTK_SERIAL present, NTRIP caster reachable"
fi

echo "  xArm ..."
if pingable "$XARM_IP"; then pass "xArm" "$XARM_IP answers ping"
else fail "xArm" "$XARM_IP does not answer ping"; fi

echo "  Velodyne ..."
if ! pingable "$LIDAR_IP"; then
  fail "Velodyne" "$LIDAR_IP does not answer ping"
else
  # Ping only proves it's powered. Data packets prove it's spinning and
  # sending to us.
  python3 - "$LIDAR_PORT" <<'EOF'
import socket, sys
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
try:
    s.bind(("", int(sys.argv[1])))
except OSError:
    sys.exit(2)
s.settimeout(3.0)
try:
    s.recvfrom(2048)
except socket.timeout:
    sys.exit(1)
EOF
  case $? in
    0) pass "Velodyne" "$LIDAR_IP answers ping, data arriving on UDP $LIDAR_PORT" ;;
    2) warn "Velodyne" "pings, but UDP $LIDAR_PORT is already in use (another driver running?)" ;;
    *) fail "Velodyne" "pings, but no data on UDP $LIDAR_PORT in 3 s (not spinning?)" ;;
  esac
fi

echo "  Arduino ..."
if [ -e "$ARDUINO" ]; then pass "Arduino" "$ARDUINO -> $(readlink -f "$ARDUINO")"
else fail "Arduino" "$ARDUINO missing (unplugged? ls /dev/ttyACM*)"; fi

echo "  CAN ..."
if [ ! -e "/sys/class/net/$CAN_IF" ]; then
  fail "CAN" "$CAN_IF not found (Kvaser unplugged?)"
else
  drv=$(basename "$(readlink -f "/sys/class/net/$CAN_IF/device/driver")")
  det=$(ip -details link show "$CAN_IF")
  state=$(grep -oP 'can state \K[A-Z-]+' <<<"$det")
  br=$(grep -oP 'bitrate \K[0-9]+' <<<"$det" | head -1)
  rms=$(grep -oP 'restart-ms \K[0-9]+' <<<"$det")
  if [ "$drv" != kvaser_usb ]; then
    kv=$(for i in /sys/class/net/can*; do
           [ "$(basename "$(readlink -f "$i/device/driver")")" = kvaser_usb ] && basename "$i"
         done)
    fail "CAN" "$CAN_IF is not the Kvaser (driver $drv). Kvaser is: ${kv:-not found}"
  elif ! grep -q 'state UP' <<<"$det" || [ "$state" != ERROR-ACTIVE ]; then
    fail "CAN" "$CAN_IF is not healthy: link $(grep -oP 'state \K[A-Z]+' <<<"$det" | head -1), can state ${state:-?}"
  elif [ "$br" != "$CAN_BITRATE" ] || [ "$rms" != "$CAN_RESTART_MS" ]; then
    fail "CAN" "$CAN_IF bitrate $br restart-ms $rms (want $CAN_BITRATE / $CAN_RESTART_MS)"
  else
    pass "CAN" "$CAN_IF Kvaser UP, ERROR-ACTIVE, $br bps, restart-ms $rms"
    # Passive listen only: transmits nothing, moves nothing.
    echo "  LINAKs (listening on $CAN_IF for 3 s) ..."
    can_out=$(timeout 30 python3 "$REPO/src/planting/unit_test/can_tests/can_bus_check.py" \
                --channel "$CAN_IF" --no-probe --listen 3 2>&1)
    if [ $? -eq 0 ]; then
      pass "LINAKs" "auger 0x20 and chute 0x21 heard on $CAN_IF"
    else
      fail "LINAKs" "not both heard on $CAN_IF (details below the summary)"
      LINAK_DETAIL=$(sed -n '/Verdict/,$p' <<<"$can_out")
      [ -z "$LINAK_DETAIL" ] && LINAK_DETAIL=$(tail -15 <<<"$can_out")
    fi
  fi
fi

header "Summary"
printf '  %s\n' "${RESULTS[@]}"
[ -n "${LINAK_DETAIL:-}" ] && { echo; echo "  LINAK check output:"; sed 's/^/    /' <<<"$LINAK_DETAIL"; }
[ "$SETUP_ERRORS" -gt 0 ] && echo "  ${Y}$SETUP_ERRORS setup command(s) failed, see step 1 above.${N}"
echo
if [ "$FAILS" -gt 0 ]; then
  echo "${R}${B}$FAILS check(s) failed.${N} Fix them and re-run (the desktop has not been touched)."
  $FORCE || exit 1
  echo "${Y}--force given: continuing anyway.${N}"
else
  if [ "$WARNS" -gt 0 ]; then
    echo "${G}${B}All checks passed${N} ($WARNS warning(s), see above)."
  else
    echo "${G}${B}All checks passed.${N}"
  fi
fi

$CHECK_ONLY && exit 0

# ── 3. Free the CPU ──────────────────────────────────────────────────────────
if ! $KEEP_UI; then
  header "3. Stopping the desktop and background services"
  run sudo systemctl stop apt-daily.timer apt-daily-upgrade.timer packagekit.service \
                          snapd.service snapd.socket
  run sudo systemctl isolate multi-user.target
  sleep 3

  # Apps that outlive the desktop, or were never part of it (VS Code Remote-SSH).
  echo "  closing VS Code, VS Code server and Firefox if still running"
  pkill -u "$USER" -x code 2>/dev/null
  pkill -u "$USER" -f '\.vscode-server/' 2>/dev/null
  pkill -u "$USER" -f '/snap/firefox/' 2>/dev/null
  sleep 2
  pkill -KILL -u "$USER" -x code 2>/dev/null
  pkill -KILL -u "$USER" -f '\.vscode-server/' 2>/dev/null
  pkill -KILL -u "$USER" -f '/snap/firefox/' 2>/dev/null

  # Pages swapped out while the desktop was running stay in swap and get pulled
  # back in slowly mid-test. Move them back to RAM now if there's room.
  swap_used=$(free -m | awk '/^Swap:/ {print $3}')
  mem_avail=$(free -m | awk '/^Mem:/ {print $7}')
  if [ "$swap_used" -gt 100 ] && [ "$mem_avail" -gt $((swap_used + 1024)) ]; then
    echo "  moving ${swap_used} MB from swap back to RAM (can take a minute)"
    run sudo swapoff -a && run sudo swapon -a
  fi
fi

# ── 4. CPU and RAM ───────────────────────────────────────────────────────────
header "4. CPU and RAM before launch"
read -r _ u n s i w q sq st _ < /proc/stat
t1=$((u + n + s + i + w + q + sq + st)); i1=$((i + w))
sleep 3
read -r _ u n s i w q sq st _ < /proc/stat
t2=$((u + n + s + i + w + q + sq + st)); i2=$((i + w))
cpu=$(( 100 * ((t2 - t1) - (i2 - i1)) / (t2 - t1) ))
cores=$(nproc)
echo "  CPU:  ${cpu}% of $cores cores (about $(awk "BEGIN {printf \"%.1f\", $cpu * $cores / 100}") cores busy)"
echo "  Load: $(cut -d' ' -f1-3 /proc/loadavg)  (1 / 5 / 15 min)"
free -m | awk '/^Mem:/  {printf "  RAM:  %d MB used of %d MB, %d MB available\n", $3, $2, $7}
               /^Swap:/ {printf "  Swap: %d MB used\n", $3}'
temps=$(timeout 2 tegrastats --interval 1000 2>/dev/null | head -1 | grep -oE '(cpu|gpu|tj)@[0-9.]+C' | tr '\n' ' ')
[ -n "$temps" ] && echo "  Temp: $temps"
echo
echo "  Top processes:"
top -b -n 2 -d 2 -o %CPU -w 120 | awk '/^top -/ {n++} n == 2' | sed -n '7,15p' | sed 's/^/  /'

# ── 5. Launch ────────────────────────────────────────────────────────────────
header "5. Launching foxglove_operator"
ts_ip=$(tailscale ip -4 2>/dev/null | head -1)
echo "  Foxglove: connect to ws://${ts_ip:-<jetson-ip>}:8765"
echo "  Stop with Ctrl-C. Afterwards run scripts/posttest.sh to bring the desktop back."
echo
cd ~/dev/ros2_ws || exit 1
# shellcheck disable=SC1091
source install/setup.bash
exec ros2 launch "$LAUNCH_FILE"