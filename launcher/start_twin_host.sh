#!/bin/bash
# =============================================================================
# start_twin_host.sh  (NEW FILE - ROS repo: ~/VECTOR/launcher/start_twin_host.sh)
#
# Runs the VECTOR twin host on the LAB PC, so twin apps on other computers
# (Windows, Mac, Linux without ROS) can use this PC's ROS 2 and cars.
#
#   bash ~/VECTOR/launcher/start_twin_host.sh                  run it (keep the window open)
#   bash ~/VECTOR/launcher/start_twin_host.sh install          add a "VECTOR Twin Host" icon
#   bash ~/VECTOR/launcher/start_twin_host.sh install --autostart
#                                                              ...and start it at every login
# Any other arguments are passed to twin_host (e.g. --wheel-ws ~/Desktop/adham_ws).
# =============================================================================

WS="${VECTOR_WS:-$HOME/VECTOR}"
SELF="$(realpath "$0")"
LOG_DIR="$HOME/.vector_twin/host_logs"

# ---------------------------------------------------------------- install
if [ "$1" = "install" ]; then
    APP_DIR="$HOME/.local/share/applications"
    APP="$APP_DIR/vector-twin-host.desktop"
    mkdir -p "$APP_DIR"
    cat > "$APP" <<EOF
[Desktop Entry]
Type=Application
Name=VECTOR Twin Host
Comment=Share this lab PC's ROS 2 and cars with VECTOR twins on other computers
Exec=bash "$SELF"
Icon=network-server
Terminal=true
Categories=Science;Education;
EOF
    chmod +x "$APP"

    DESK="$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")"
    mkdir -p "$DESK"
    cp "$APP" "$DESK/vector-twin-host.desktop"
    chmod +x "$DESK/vector-twin-host.desktop"
    gio set "$DESK/vector-twin-host.desktop" metadata::trusted true 2>/dev/null || true
    echo "Icon added: $DESK/vector-twin-host.desktop (and in the app menu)"

    if [ "$2" = "--autostart" ]; then
        mkdir -p "$HOME/.config/autostart" "$LOG_DIR"
        cat > "$HOME/.config/autostart/vector-twin-host.desktop" <<EOF
[Desktop Entry]
Type=Application
Name=VECTOR Twin Host (autostart)
Exec=bash -c 'bash "$SELF" > "$LOG_DIR/twin_host.log" 2>&1'
Terminal=false
X-GNOME-Autostart-enabled=true
EOF
        echo "Autostart on: the host starts at every login (log: $LOG_DIR/twin_host.log)"
    fi
    exit 0
fi

# ---------------------------------------------------------------- run
fail() {
    echo "ERROR: $1"
    read -r -p "Press Enter to close..." _ 2>/dev/null
    exit 1
}

for d in jazzy humble iron rolling; do
    if [ -f "/opt/ros/$d/setup.bash" ]; then
        source "/opt/ros/$d/setup.bash"
        break
    fi
done
[ -n "$ROS_DISTRO" ] || fail "ROS 2 not found in /opt/ros."

[ -f "$WS/install/setup.bash" ] || fail "$WS is not built. Run: cd $WS && colcon build --symlink-install"
source "$WS/install/setup.bash"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

ros2 pkg executables vector_twin_tools 2>/dev/null | grep -q twin_host \
    || fail "twin_host is not in the workspace yet. Run: cd $WS && colcon build --symlink-install"

echo "Keep this window open while others use the twin. Ctrl+C stops the host."
exec ros2 run vector_twin_tools twin_host "$@"
