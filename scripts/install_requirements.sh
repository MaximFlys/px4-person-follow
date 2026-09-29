#!/usr/bin/env bash
# Install everything required to recreate this project:
# OpenCV, the YOLO11n ONNX model, ros2_yolos_cpp, PX4 SITL, Gazebo Harmonic,
# and the Micro XRCE-DDS agent.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
YOLO_WS="${YOLO_WS:-$HOME/ros2_ws}"
PX4_DIR="${PX4_DIR:-$HOME/src/PX4-Autopilot}"
AGENT_DIR="${AGENT_DIR:-$HOME/src/Micro-XRCE-DDS-Agent}"
PX4_REF="${PX4_REF:-1403709f65}"
PX4_MSGS_REF="ca9895d26b88ffc14209daf2fc8d8564e51adddd"
ROS_SETUP="/opt/ros/jazzy/setup.bash"

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ROS 2 Jazzy is not installed at ${ROS_SETUP}." >&2
  echo "Install it from https://docs.ros.org/en/jazzy/Installation.html and run this script again." >&2
  exit 1
fi

echo "Installing OpenCV, cv_bridge, and the ROS packages the pipeline imports."
sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-vision-msgs \
  ros-jazzy-cv-bridge \
  libopencv-dev \
  ros-jazzy-ros-gz-image \
  ros-jazzy-rqt-image-view \
  python3-numpy \
  python3-pip \
  python3-colcon-common-extensions \
  git \
  cmake \
  build-essential

echo "Checking out px4_msgs ${PX4_MSGS_REF}."
if [[ ! -f "${ROOT}/src/px4_msgs/package.xml" ]]; then
  rm -rf "${ROOT}/src/px4_msgs"
  git clone https://github.com/PX4/px4_msgs.git "${ROOT}/src/px4_msgs"
fi
git -C "${ROOT}/src/px4_msgs" fetch --tags origin
git -C "${ROOT}/src/px4_msgs" checkout "${PX4_MSGS_REF}"

echo "Building flight_control_node and px4_msgs."
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
set -u
cd "$ROOT"
colcon build --packages-select px4_msgs flight_control_node

echo "Building ros2_yolos_cpp (OpenCV is required; ONNX Runtime 1.20.1 is downloaded if missing)."
mkdir -p "${YOLO_WS}/src" "${YOLO_WS}/models"
if [[ ! -d "${YOLO_WS}/src/ros2_yolos_cpp/.git" ]]; then
  git clone https://github.com/Geekgineer/ros2_yolos_cpp.git "${YOLO_WS}/src/ros2_yolos_cpp"
fi
cp "${ROOT}/models/coco.names" "${YOLO_WS}/models/coco.names"
set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
set -u
cd "$YOLO_WS"
colcon build --packages-select ros2_yolos_cpp --cmake-args -DCMAKE_BUILD_TYPE=Release

if [[ ! -f "${YOLO_WS}/models/yolo11n.onnx" ]]; then
  echo "Exporting YOLO11n to ONNX. This downloads yolo11n.pt."
  python3 -m pip install --user ultralytics
  (
    cd "${YOLO_WS}/models"
    python3 - <<'PY'
from pathlib import Path
import shutil
from ultralytics import YOLO

dest = Path.cwd() / "yolo11n.onnx"
exported = Path(YOLO("yolo11n.pt").export(format="onnx", half=True))
if exported.resolve() != dest.resolve():
    shutil.copy2(exported, dest)
print(dest)
PY
  )
else
  echo "YOLO11n ONNX already present: ${YOLO_WS}/models/yolo11n.onnx"
fi

if ! command -v MicroXRCEAgent >/dev/null 2>&1; then
  echo "Building Micro XRCE-DDS Agent."
  if [[ ! -d "${AGENT_DIR}/.git" ]]; then
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git "$AGENT_DIR"
  fi
  cmake -S "$AGENT_DIR" -B "${AGENT_DIR}/build"
  cmake --build "${AGENT_DIR}/build" -j"$(nproc)"
  sudo cmake --install "${AGENT_DIR}/build"
  sudo ldconfig
fi

if [[ ! -d "${PX4_DIR}/.git" ]]; then
  echo "Cloning PX4-Autopilot at ${PX4_REF}."
  git clone --recursive https://github.com/PX4/PX4-Autopilot.git "$PX4_DIR"
  git -C "$PX4_DIR" checkout "$PX4_REF"
fi

if ! command -v gz >/dev/null 2>&1; then
  echo "Installing the PX4 Ubuntu packages, including Gazebo Harmonic."
  bash "${PX4_DIR}/Tools/setup/ubuntu.sh" --no-nuttx
fi

echo
echo "Requirements are installed."
echo "  OpenCV:          $(pkg-config --modversion opencv4 2>/dev/null || echo 'installed, pkg-config name not opencv4')"
echo "  YOLO model:      ${YOLO_WS}/models/yolo11n.onnx"
echo "  Class names:     ${YOLO_WS}/models/coco.names"
echo "  Detector:        ${YOLO_WS}/src/ros2_yolos_cpp"
echo "  Flight package:  ${ROOT}"
echo "Run ./scripts/run_gazebo_sim.sh from ${ROOT} to start the simulation."
