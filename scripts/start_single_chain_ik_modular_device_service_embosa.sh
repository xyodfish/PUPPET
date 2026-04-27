#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-${REPO_ROOT}/build}"

RUNTIME_CFG="${1:-${REPO_ROOT}/config/runtime/demo_single_chain_ik_runtime_modular.yaml}"
DEVICE_CFG="${2:-${REPO_ROOT}/config/device/device_service_single_chain_ik_embosa.yaml}"
VIEWER_CFG="${3:-${REPO_ROOT}/config/tools/demo_single_chain_ik_visualizer.yaml}"

RUNTIME_BIN="${BUILD_DIR}/app/cpp/runtime/teleop_runtime_embosa_main"
DEVICE_BIN="${BUILD_DIR}/app/cpp/devices/device_service"
VIEWER_BIN="${BUILD_DIR}/app/cpp/tools/retargeting_mujoco_visualizer"

for f in "${RUNTIME_BIN}" "${DEVICE_BIN}" "${VIEWER_BIN}"; do
  if [[ ! -x "${f}" ]]; then
    echo "[start_single_chain_ik_modular_device_service_embosa] missing executable: ${f}" >&2
    echo "Build first: cmake -S . -B build && cmake --build build -j\$(nproc)" >&2
    exit 1
  fi
done

mkdir -p "${REPO_ROOT}/bin/log"
RUNTIME_LOG="${REPO_ROOT}/bin/log/single_chain_ik_runtime_modular_embosa.log"
DEVICE_LOG="${REPO_ROOT}/bin/log/single_chain_ik_device_service_embosa.log"
VIEWER_LOG="${REPO_ROOT}/bin/log/single_chain_ik_viewer_embosa.log"

pids=()
cleanup() {
  echo "[start_single_chain_ik_modular_device_service_embosa] stopping nodes..."
  for pid in "${pids[@]:-}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
  wait || true
}
trap cleanup EXIT INT TERM

echo "[start_single_chain_ik_modular_device_service_embosa] start runtime"
"${RUNTIME_BIN}" "${RUNTIME_CFG}" >"${RUNTIME_LOG}" 2>&1 &
pids+=("$!")
sleep 1

echo "[start_single_chain_ik_modular_device_service_embosa] start device service"
"${DEVICE_BIN}" "${DEVICE_CFG}" >"${DEVICE_LOG}" 2>&1 &
pids+=("$!")
sleep 1

echo "[start_single_chain_ik_modular_device_service_embosa] start viewer"
"${VIEWER_BIN}" "${VIEWER_CFG}" >"${VIEWER_LOG}" 2>&1 &
pids+=("$!")

echo "[start_single_chain_ik_modular_device_service_embosa] running"
echo "  runtime log: ${RUNTIME_LOG}"
echo "  device  log: ${DEVICE_LOG}"
echo "  viewer  log: ${VIEWER_LOG}"
echo "Press Ctrl+C to stop all."

wait
