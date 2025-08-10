#!/usr/bin/env bash
set -euo pipefail

### === Config (edit as needed) ===
ROS_DISTRO="${ROS_DISTRO:-humble}"                     # e.g., humble, jazzy
ROS_WS="${ROS_WS:-$HOME/ros2_ws}"                      # path to your ROS 2 workspace
SERIAL_DEV="${SERIAL_DEV:-/dev/ttyUSB0}"               # micro-ROS serial device

# Teleop launch config
TELEOP_PKG="${TELEOP_PKG:-p9n_bringup}"
TELEOP_LAUNCH="${TELEOP_LAUNCH:-teleop.launch.py}"
TELEOP_TOPIC="${TELEOP_TOPIC:-cmd_vel}"
TELEOP_HW="${TELEOP_HW:-DualSense}"
TELEOP_LINEAR="${TELEOP_LINEAR:-1.0}"
TELEOP_ANGULAR="${TELEOP_ANGULAR:-1.0}"

# Delay before starting teleop (lets the agent come up)
TELEOP_DELAY="${TELEOP_DELAY:-3}"

# Preferred terminal (auto-detected if empty)
PREFERRED_TERM="${PREFERRED_TERM:-}"

### === Detect a terminal to use ===
pick_terminal() {
  local t
  if [[ -n "${PREFERRED_TERM}" ]] && command -v "${PREFERRED_TERM}" >/dev/null 2>&1; then
    echo "$(command -v "${PREFERRED_TERM}")"
    return
  fi
  for t in gnome-terminal xfce4-terminal kitty konsole xterm; do
    if command -v "$t" >/dev/null 2>&1; then
      echo "$(command -v "$t")"
      return
    fi
  done
  echo "ERROR"
}

TERM_BIN="$(pick_terminal)"
if [[ "${TERM_BIN}" == "ERROR" ]]; then
  echo "No supported terminal found (gnome-terminal/xfce4-terminal/kitty/konsole/xterm)."
  echo "Install one (e.g., 'sudo apt install gnome-terminal') or set PREFERRED_TERM to a valid terminal."
  exit 1
fi

### === Build ExecStart line per terminal ===
# Each variant runs a login shell, sources ROS envs, runs command, and keeps the window open.
make_execstart() {
  local title="$1"
  local cmd="$2"

  case "$(basename "${TERM_BIN}")" in
    gnome-terminal)
      # gnome-terminal supports: --title and runs bash -lc
      echo "${TERM_BIN} --title=\"${title}\" -- bash -lc '${cmd}; exec bash'"
      ;;
    xfce4-terminal)
      # xfce4-terminal: --title, -e is deprecated; use --command with /bin/bash -lc
      echo "${TERM_BIN} --title \"${title}\" --command bash -lc '${cmd}; exec bash'"
      ;;
    kitty)
      # kitty: set title and run bash -lc
      echo "${TERM_BIN} --title \"${title}\" bash -lc '${cmd}; exec bash'"
      ;;
    konsole)
      # konsole: --hold to keep open, -e to execute
      echo "${TERM_BIN} --hold --profile Default --title \"${title}\" -e bash -lc '${cmd}; exec bash'"
      ;;
    xterm)
      # xterm: -hold keeps the window open, -T sets title, -e runs command
      echo "${TERM_BIN} -hold -T \"${title}\" -e bash -lc '${cmd}; exec bash'"
      ;;
    *)
      # Fallback behaves like xterm
      echo "${TERM_BIN} -hold -T \"${title}\" -e bash -lc '${cmd}; exec bash'"
      ;;
  esac
}

### === Compose the commands to run inside the terminals ===
ROS_SOURCE_CMDS="source /opt/ros/${ROS_DISTRO}/setup.bash; if [ -f \"${ROS_WS}/install/setup.bash\" ]; then source \"${ROS_WS}/install/setup.bash\"; fi"

AGENT_INNER_CMD="${ROS_SOURCE_CMDS}; ros2 run micro_ros_agent micro_ros_agent serial --dev ${SERIAL_DEV}"
TELEOP_INNER_CMD="${ROS_SOURCE_CMDS}; sleep ${TELEOP_DELAY}; ros2 launch ${TELEOP_PKG} ${TELEOP_LAUNCH} topic_name:=${TELEOP_TOPIC} hw_type:=${TELEOP_HW} linear_speed:=${TELEOP_LINEAR} angular_speed:=${TELEOP_ANGULAR}"
MONITOR_INNER_CMD="${ROS_SOURCE_CMDS}; echo '--- ROS 2 Node list ---'; ros2 node list; echo; echo '--- ROS 2 Topic list ---'; ros2 topic list; echo; echo '--- Live cmd_vel ---'; ros2 topic echo /cmd_vel"

AGENT_EXECSTART="$(make_execstart "micro-ROS Agent" "${AGENT_INNER_CMD}")"
TELEOP_EXECSTART="$(make_execstart "Teleop" "${TELEOP_INNER_CMD}")"
MONITOR_EXECSTART="$(make_execstart "ROS2 Monitor" "${MONITOR_INNER_CMD}")"

### === Create user systemd unit files ===
UNIT_DIR="${HOME}/.config/systemd/user"
mkdir -p "${UNIT_DIR}"

AGENT_UNIT="${UNIT_DIR}/micro-ros-agent-term.service"
TELEOP_UNIT="${UNIT_DIR}/teleop-term.service"
MONITOR_UNIT="${UNIT_DIR}/ros2-monitor-term.service"

cat > "${AGENT_UNIT}" <<EOF
[Unit]
Description=micro-ROS Agent in a visible terminal
After=graphical-session.target
PartOf=graphical-session.target

[Service]
Type=simple
ExecStart=${AGENT_EXECSTART}
Restart=on-failure

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
Restart=on-failure

[Install]
WantedBy=graphical-session.target
EOF

cat > "${MONITOR_UNIT}" <<EOF
[Unit]
Description=ROS 2 Monitor in a visible terminal
After=graphical-session.target teleop-term.service
Wants=teleop-term.service
PartOf=graphical-session.target

[Service]
Type=simple
ExecStart=${MONITOR_EXECSTART}
Restart=always

[Install]
WantedBy=graphical-session.target
EOF

### === Enable and start the user services ===
systemctl --user daemon-reload
systemctl --user enable micro-ros-agent-term.service
systemctl --user enable teleop-term.service
systemctl --user enable ros2-monitor-term.service

# Start them now (they'll also auto-start after you log into the desktop)
systemctl --user start micro-ros-agent-term.service
systemctl --user start teleop-term.service
systemctl --user start ros2-monitor-term.service

echo
echo "Done!"
echo "Two user services were created and started:"
echo "  - micro-ros-agent-term.service"
echo "  - teleop-term.service"
echo
echo "They will launch visible terminals after you log into your graphical session."
echo "Check status with:"
echo "  systemctl --user status micro-ros-agent-term.service"
echo "  systemctl --user status teleop-term.service"
