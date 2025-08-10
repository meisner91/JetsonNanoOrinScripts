#!/usr/bin/env bash
set -euo pipefail

### ===== Config (edit as needed) =====
ROS_DISTRO="${ROS_DISTRO:-humble}"            # e.g. humble, jazzy
ROS_WS="${ROS_WS:-$HOME/ros2_ws}"             # your workspace
SERIAL_DEV="${SERIAL_DEV:-/dev/ttyUSB0}"

TELEOP_PKG="${TELEOP_PKG:-p9n_bringup}"
TELEOP_LAUNCH="${TELEOP_LAUNCH:-teleop.launch.py}"
TELEOP_TOPIC="${TELEOP_TOPIC:-cmd_vel}"
TELEOP_HW="${TELEOP_HW:-DualSense}"
TELEOP_LINEAR="${TELEOP_LINEAR:-1.0}"
TELEOP_ANGULAR="${TELEOP_ANGULAR:-1.0}"
TELEOP_DELAY="${TELEOP_DELAY:-3}"             # seconds

PREFERRED_TERM="${PREFERRED_TERM:-}"          # leave empty to auto-detect

### ===== Service names/paths =====
UNIT_DIR="${HOME}/.config/systemd/user"
AGENT_UNIT="${UNIT_DIR}/micro-ros-agent-term.service"
TELEOP_UNIT="${UNIT_DIR}/teleop-term.service"
MONITOR_UNIT="${UNIT_DIR}/ros2-monitor-term.service"

### ===== Helpers =====
pick_terminal() {
  local t
  if [[ -n "${PREFERRED_TERM}" ]] && command -v "${PREFERRED_TERM}" >/dev/null 2>&1; then
    command -v "${PREFERRED_TERM}"; return
  fi
  for t in gnome-terminal xfce4-terminal kitty konsole xterm; do
    if command -v "$t" >/dev/null 2>&1; then
      command -v "$t"; return
    fi
  done
  echo "ERROR"
}

TERM_BIN="$(pick_terminal)"
if [[ "${TERM_BIN}" == "ERROR" ]]; then
  echo "No supported terminal found (gnome-terminal/xfce4-terminal/kitty/konsole/xterm)."
  echo "Install one (e.g., sudo apt install gnome-terminal) or set PREFERRED_TERM."
  exit 1
fi

make_execstart() {
  local title="$1"
  local cmd="$2"
  case "$(basename "${TERM_BIN}")" in
    gnome-terminal)
      # --wait prevents duplicate respawns (systemd stays active while terminal is open)
      echo "${TERM_BIN} --title=\"${title}\" --wait -- bash -lc '${cmd}; exec bash'"
      ;;
    xfce4-terminal)
      echo "${TERM_BIN} --title \"${title}\" --command bash -lc '${cmd}; exec bash'"
      ;;
    kitty)
      echo "${TERM_BIN} --title \"${title}\" bash -lc '${cmd}; exec bash'"
      ;;
    konsole)
      echo "${TERM_BIN} --hold --title \"${title}\" -e bash -lc '${cmd}; exec bash'"
      ;;
    xterm)
      echo "${TERM_BIN} -hold -T \"${title}\" -e bash -lc '${cmd}; exec bash'"
      ;;
    *)
      echo "${TERM_BIN} -hold -T \"${title}\" -e bash -lc '${cmd}; exec bash'"
      ;;
  esac
}

### ===== Commands run inside terminals =====
ROS_SOURCE_CMDS="source /opt/ros/${ROS_DISTRO}/setup.bash; if [ -f \"${ROS_WS}/install/setup.bash\" ]; then source \"${ROS_WS}/install/setup.bash\"; fi"

AGENT_INNER_CMD="${ROS_SOURCE_CMDS}; ros2 run micro_ros_agent micro_ros_agent serial --dev ${SERIAL_DEV}"
TELEOP_INNER_CMD="${ROS_SOURCE_CMDS}; sleep ${TELEOP_DELAY}; ros2 launch ${TELEOP_PKG} ${TELEOP_LAUNCH} topic_name:=${TELEOP_TOPIC} hw_type:=${TELEOP_HW} linear_speed:=${TELEOP_LINEAR} angular_speed:=${TELEOP_ANGULAR}"
MONITOR_INNER_CMD="${ROS_SOURCE_CMDS}; echo '--- ROS 2 Node list ---'; ros2 node list || true; echo; echo '--- ROS 2 Topic list ---'; ros2 topic list || true; echo; echo '--- Live ${TELEOP_TOPIC} ---'; ros2 topic echo /${TELEOP_TOPIC} geometry_msgs/msg/Twist"

AGENT_EXECSTART="$(make_execstart "micro-ROS Agent" "${AGENT_INNER_CMD}")"
TELEOP_EXECSTART="$(make_execstart "Teleop" "${TELEOP_INNER_CMD}")"
MONITOR_EXECSTART="$(make_execstart "ROS2 Monitor" "${MONITOR_INNER_CMD}")"

### ===== Clean up any old autostart .desktop (prevents double starts) =====
mkdir -p ~/.config/autostart
rm -f ~/.config/autostart/micro_ros_agent.desktop \
      ~/.config/autostart/teleop.desktop \
      ~/.config/autostart/ros2-monitor.desktop || true

### ===== Stop/disable old services & remove old units (idempotent) =====
for svc in micro-ros-agent-term.service teleop-term.service ros2-monitor-term.service; do
  systemctl --user stop "$svc" 2>/dev/null || true
  systemctl --user disable "$svc" 2>/dev/null || true
done

mkdir -p "${UNIT_DIR}"
rm -f "${AGENT_UNIT}" "${TELEOP_UNIT}" "${MONITOR_UNIT}" 2>/dev/null || true
systemctl --user daemon-reload || true
systemctl --user reset-failed || true

### ===== Create fresh unit files =====
cat > "${AGENT_UNIT}" <<EOF
[Unit]
Description=micro-ROS Agent in a visible terminal
After=graphical-session.target
PartOf=graphical-session.target

[Service]
Type=simple
ExecStart=${AGENT_EXECSTART}
Restart=no
StartLimitIntervalSec=60
StartLimitBurst=5
RemainAfterExit=no

[Install]
WantedBy=graphical-session.target
EOF

cat > "${TELEOP_UNIT}" <<EOF
[Unit]
Description=Teleop launch in a visible terminal
After=graphical-session.target micro-ros-agent-term.service
Wants=micro-ros-agent-term.service
PartOf=graphical-session.target

[Service]
Type=simple
ExecStart=${TELEOP_EXECSTART}
Restart=no
StartLimitIntervalSec=60
StartLimitBurst=5
RemainAfterExit=no

[Install]
WantedBy=graphical-session.target
EOF

cat > "${MONITOR_UNIT}" <<EOF
[Unit]
Description=ROS 2 Monitor (nodes, topics, live cmd_vel) in a visible terminal
After=graphical-session.target teleop-term.service
Wants=teleop-term.service
PartOf=graphical-session.target

[Service]
Type=simple
ExecStart=${MONITOR_EXECSTART}
Restart=no
StartLimitIntervalSec=60
StartLimitBurst=5
RemainAfterExit=no

[Install]
WantedBy=graphical-session.target
EOF

### ===== Enable and start (single instance each) =====
systemctl --user daemon-reload
systemctl --user enable micro-ros-agent-term.service
systemctl --user enable teleop-term.service
systemctl --user enable ros2-monitor-term.service

# Start them now (they'll also auto-open after you log into GNOME)
systemctl --user start micro-ros-agent-term.service
systemctl --user start teleop-term.service
systemctl --user start ros2-monitor-term.service

echo
echo "✅ Setup complete. Three user services created, enabled, and started:"
echo "   - micro-ros-agent-term.service"
echo "   - teleop-term.service"
echo "   - ros2-monitor-term.service"
echo
echo "Check status:"
echo "  systemctl --user status micro-ros-agent-term.service"
echo "  systemctl --user status teleop-term.service"
echo "  systemctl --user status ros2-monitor-term.service"
