#!/usr/bin/env bash
# Open the Gazebo Harmonic + PX4 SITL stack in one gnome-terminal window.
# Each tab is one of the terminals that used to be pasted by hand.
# The flight node starts only after the YOLO detector is active.
# Closing the PX4 tab stops the simulator.

set -o pipefail

PX4_DIR="${PX4_DIR:-$HOME/src/PX4-Autopilot}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
YOLO_WS="${YOLO_WS:-$HOME/ros2_ws}"
SUPER_WS="${SUPER_WS:-$HOME/super_ws}"
CAMERA_TOPIC="/world/baylands/model/x500_mono_cam_0/link/camera_link/sensor/camera/image"
YOLO_MODEL="${YOLO_MODEL:-$YOLO_WS/models/yolo11n.onnx}"
YOLO_LABELS="${YOLO_LABELS:-$YOLO_WS/models/coco.names}"
LOG_DIR="${SUPER_WS}/log/gazebo_sim"
READY_WAIT_SECONDS=300

SCRIPT="$(readlink -f "$0")"

hold() {
  local status=$?
  echo
  if [[ "$status" -ne 0 ]]; then
    echo "Exited with status ${status}."
  fi
  echo "Press Enter to close this tab."
  read -r _
}

source_ros() {
  set +u
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
}

wait_for_px4_ready() {
  echo "Waiting for PX4 to report ready (up to ${READY_WAIT_SECONDS}s)..."
  local i
  for ((i = 0; i < READY_WAIT_SECONDS; i++)); do
    if [[ -f "${LOG_DIR}/px4.log" ]] && grep -q "Ready for takeoff" "${LOG_DIR}/px4.log"; then
      echo "PX4 is ready."
      return 0
    fi
    sleep 1
  done
  echo "PX4 did not report ready. Later tabs were not started."
  return 1
}

role_px4() {
  cd "$PX4_DIR"
  export PX4_GZ_WORLD=baylands
  export QT_QPA_PLATFORM=xcb
  echo "Gazebo Harmonic world: baylands"
  echo "Airframe: x500_mono_cam"
  stdbuf -oL -eL make px4_sitl gz_x500_mono_cam 2>&1 | tee "${LOG_DIR}/px4.log"
  hold
}

role_agent() {
  echo "uXRCE-DDS agent on UDP port 8888 (PX4 <-> ROS 2)."
  MicroXRCEAgent udp4 -p 8888
  hold
}

role_bridge() {
  source_ros
  wait_for_px4_ready || { hold; return; }
  echo "Bridging Gazebo camera image onto a ROS 2 topic."
  ros2 run ros_gz_image image_bridge "$CAMERA_TOPIC"
  hold
}

role_view() {
  source_ros
  export QT_QPA_PLATFORM=xcb
  wait_for_px4_ready || { hold; return; }
  echo "Camera viewer. Topic: ${CAMERA_TOPIC}"
  ros2 run rqt_image_view rqt_image_view "$CAMERA_TOPIC"
  hold
}

role_yolo() {
  set +u
  # shellcheck disable=SC1091
  source "${YOLO_WS}/install/setup.bash"
  set +u
  wait_for_px4_ready || { hold; return; }
  echo "YOLO detector on ${CAMERA_TOPIC}"
  ros2 launch ros2_yolos_cpp detector.launch.py \
    "model_path:=${YOLO_MODEL}" \
    "labels_path:=${YOLO_LABELS}" \
    "image_topic:=${CAMERA_TOPIC}"
  hold
}

role_lifecycle() {
  set +u
  # shellcheck disable=SC1091
  source "${YOLO_WS}/install/setup.bash"
  set +u
  rm -f "${LOG_DIR}/detector.active"
  echo "Waiting for /yolos_detector..."
  local i
  for ((i = 0; i < READY_WAIT_SECONDS; i++)); do
    if ros2 lifecycle get /yolos_detector >/dev/null 2>&1; then
      break
    fi
    sleep 1
  done
  if ! ros2 lifecycle get /yolos_detector >/dev/null 2>&1; then
    echo "/yolos_detector did not appear."
    hold
    return
  fi
  echo "Loading weights and configuring /yolos_detector."
  ros2 lifecycle set /yolos_detector configure || { hold; return; }
  echo "Activating /yolos_detector."
  ros2 lifecycle set /yolos_detector activate || { hold; return; }
  touch "${LOG_DIR}/detector.active"
  echo "Detector is active."
  hold
}

role_flight() {
  source_ros
  set +u
  # shellcheck disable=SC1091
  source "${SUPER_WS}/install/setup.bash"
  set +u
  echo "Waiting for the detector to become active..."
  local i
  for ((i = 0; i < READY_WAIT_SECONDS; i++)); do
    if [[ -f "${LOG_DIR}/detector.active" ]]; then
      break
    fi
    sleep 1
  done
  if [[ ! -f "${LOG_DIR}/detector.active" ]]; then
    echo "Detector never became active. Flight node was not started."
    hold
    return
  fi
  echo "Starting flight_control_node. It commands offboard and arms about one second later."
  ros2 run flight_control_node start_flight
  hold
}

launch_terminals() {
  if ! command -v gnome-terminal >/dev/null 2>&1; then
    echo "gnome-terminal is required." >&2
    exit 1
  fi
  if [[ ! -d "$PX4_DIR" ]]; then
    echo "PX4 tree not found: ${PX4_DIR}" >&2
    exit 1
  fi
  if [[ ! -f "$YOLO_MODEL" || ! -f "$YOLO_LABELS" ]]; then
    echo "YOLO11n model is missing (${YOLO_MODEL} and ${YOLO_LABELS})." >&2
    echo "Run ./scripts/install_requirements.sh to install OpenCV and export the model." >&2
    exit 1
  fi
  if [[ ! -f "${YOLO_WS}/install/setup.bash" ]]; then
    echo "ros2_yolos_cpp is not built in ${YOLO_WS}." >&2
    echo "Run ./scripts/install_requirements.sh." >&2
    exit 1
  fi
  if [[ ! -f "${SUPER_WS}/install/setup.bash" ]]; then
    echo "Build this workspace first: colcon build && source install/setup.bash" >&2
    exit 1
  fi
  if pgrep -f 'px4_sitl_default/bin/px4' >/dev/null 2>&1 || pgrep -x MicroXRCEAgent >/dev/null 2>&1; then
    echo "PX4 SITL or MicroXRCEAgent is already running. Close that stack first." >&2
    exit 1
  fi

  mkdir -p "$LOG_DIR"
  rm -f "${LOG_DIR}/px4.log" "${LOG_DIR}/detector.active"

  # Separate windows. One gnome-terminal invocation cannot reliably open
  # every tab on this machine, and --tab can attach to an existing window.
  local title role
  while IFS='|' read -r title role; do
    gnome-terminal --window --title="$title" -- "$SCRIPT" "$role"
    sleep 0.2
  done <<'EOF'
1 PX4 Gazebo|px4
2 Image bridge|bridge
3 uXRCE agent|agent
4 Camera view|view
5 YOLO|yolo
6 Detector lifecycle|lifecycle
7 Flight node|flight
EOF
}

case "${1:-}" in
  px4) role_px4 ;;
  agent) role_agent ;;
  bridge) role_bridge ;;
  view) role_view ;;
  yolo) role_yolo ;;
  lifecycle) role_lifecycle ;;
  flight) role_flight ;;
  "") launch_terminals ;;
  *)
    echo "Usage: $0" >&2
    exit 1
    ;;
esac
