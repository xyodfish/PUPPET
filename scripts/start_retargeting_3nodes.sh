#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-${REPO_ROOT}/build}"

RUNTIME_CFG="${1:-${REPO_ROOT}/config/runtime/teleop_runtime_gmr.yaml}"
DEVICE_CFG="${2:-${REPO_ROOT}/config/device/device_service_static_file_embosa.yaml}"
VIEWER_CFG="${3:-${REPO_ROOT}/config/tools/retargeting_mujoco_visualizer.yaml}"

RUNTIME_BIN="${BUILD_DIR}/app/cpp/runtime/teleop_runtime_embosa_main"
DEVICE_BIN="${BUILD_DIR}/app/cpp/devices/device_service"
VIEWER_BIN="${BUILD_DIR}/app/cpp/tools/retargeting_mujoco_visualizer"

for f in "${RUNTIME_BIN}" "${DEVICE_BIN}" "${VIEWER_BIN}"; do
  if [[ ! -x "${f}" ]]; then
    echo "[start_retargeting_3nodes] missing executable: ${f}" >&2
    echo "Build first: cmake -S . -B build && cmake --build build -j\$(nproc)" >&2
    exit 1
  fi
done

mkdir -p "${REPO_ROOT}/bin/log"
RUNTIME_LOG="${REPO_ROOT}/bin/log/teleop_runtime_embosa_main.log"
DEVICE_LOG="${REPO_ROOT}/bin/log/device_service_static_file_embosa.log"
VIEWER_LOG="${REPO_ROOT}/bin/log/retargeting_mujoco_visualizer.log"

pids=()
cleanup() {
  echo "[start_retargeting_3nodes] stopping nodes..."
  for pid in "${pids[@]:-}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
  wait || true
}
trap cleanup EXIT INT TERM

echo "[start_retargeting_3nodes] start runtime"
"${RUNTIME_BIN}" "${RUNTIME_CFG}" >"${RUNTIME_LOG}" 2>&1 &
pids+=("$!")
sleep 1

echo "[start_retargeting_3nodes] start device service (static_file_replay + embosa)"
"${DEVICE_BIN}" "${DEVICE_CFG}" >"${DEVICE_LOG}" 2>&1 &
pids+=("$!")
sleep 1

echo "[start_retargeting_3nodes] start viewer"
"${VIEWER_BIN}" "${VIEWER_CFG}" >"${VIEWER_LOG}" 2>&1 &
pids+=("$!")

echo "[start_retargeting_3nodes] running"
echo "  runtime log: ${RUNTIME_LOG}"
echo "  device  log: ${DEVICE_LOG}"
echo "  viewer  log: ${VIEWER_LOG}"
echo "Press Ctrl+C to stop all."

wait
