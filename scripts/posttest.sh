#!/usr/bin/env bash
# posttest.sh - undo step 3 of pretest.sh: bring back the desktop, Bluetooth
# and the background services it stopped.
#
#   ~/dev/ros2_ws/src/canopy/scripts/posttest.sh
#
# Same as running:
#   sudo systemctl isolate graphical.target
#   sudo systemctl start bluetooth.service snapd.socket snapd.service \
#                        apt-daily.timer apt-daily-upgrade.timer
#
# Why Bluetooth needs its own start: bluetooth.service is pulled in by
# bluetooth.target, which is started when the Bluetooth adapter appears (a udev
# device unit), not by graphical.target. `isolate multi-user.target` stops it,
# and isolating back to graphical.target doesn't restart it because the adapter
# never "appears" again.
#
# packagekit, geoclue, colord, upower and fwupd also get stopped by isolate,
# but they start on their own when the desktop asks for them.
#
# The network and CAN setup from pretest.sh is left as it is.

if [ -t 1 ]; then
  G=$'\e[32m'; R=$'\e[31m'; Y=$'\e[33m'; N=$'\e[0m'
else
  G=; R=; Y=; N=
fi

[ "$EUID" -eq 0 ] && { echo "Run as your normal user, not with sudo."; exit 1; }

if pgrep -u "$USER" -f 'foxglove_operator.launch.py|canopy_launch.launch.py' >/dev/null; then
  echo "${Y}Note:${N} the robot stack is still running. That's fine, the desktop"
  echo "      will start next to it. Ctrl-C it in its tmux window when you're done."
fi

sudo -v || exit 1

run() { echo "  \$ $*"; "$@" || echo "    ${R}failed${N}"; }

run sudo systemctl isolate graphical.target
run sudo systemctl start bluetooth.service
run sudo systemctl start snapd.socket snapd.service apt-daily.timer apt-daily-upgrade.timer

echo
for u in gdm bluetooth snapd apt-daily.timer apt-daily-upgrade.timer; do
  s=$(systemctl is-active "$u")
  if [ "$s" = active ]; then c=$G; else c=$R; fi
  printf '  %-26s %s%s%s\n' "$u" "$c" "$s" "$N"
done
echo
echo "The login screen should be back on the Jetson's monitor. Log in to get"
echo "the desktop session (and its apps) back."
