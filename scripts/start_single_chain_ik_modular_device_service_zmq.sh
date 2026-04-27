#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-${REPO_ROOT}/build}"

RUNTIME_CFG="${1:-${REPO_ROOT}/config/runtime/demo_single_chain_ik_runtime_modular_zmq.yaml}"
DEVICE_CFG="${2:-${REPO_ROOT}/config/device/device_service_single_chain_ik_zmq.yaml}"
VIEWER_CFG="${3:-${REPO_ROOT}/config/tools/demo_single_chain_ik_visualizer_zmq.yaml}"

RUNTIME_BIN="${BUILD_DIR}/app/cpp/runtime/teleop_runtime_embosa_main"
DEVICE_BIN="${BUILD_DIR}/app/cpp/devices/device_service"
VIEWER_BIN="${BUILD_DIR}/app/cpp/tools/retargeting_mujoco_visualizer_zmq"

for f in "${RUNTIME_BIN}" "${DEVICE_BIN}" "${VIEWER_BIN}"; do
  if [[ ! -x "${f}" ]]; then
    echo "[start_single_chain_ik_modular_device_service_zmq] missing executable: ${f}" >&2
    echo "Build first: cmake -S . -B build && cmake --build build -j\$(nproc)" >&2
    exit 1
  fi
done

mkdir -p "${REPO_ROOT}/bin/log"
RUNTIME_LOG="${REPO_ROOT}/bin/log/single_chain_ik_runtime_modular_zmq.log"
DEVICE_LOG="${REPO_ROOT}/bin/log/single_chain_ik_device_service_zmq.log"
VIEWER_LOG="${REPO_ROOT}/bin/log/single_chain_ik_viewer_zmq.log"

pids=()
cleanup() {
  echo "[start_single_chain_ik_modular_device_service_zmq] stopping nodes..."
  for pid in "${pids[@]:-}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
  wait || true
}
trap cleanup EXIT INT TERM

check_started() {
  local pid="$1"
  local name="$2"
  local log_file="$3"
  if ! kill -0 "${pid}" 2>/dev/null; then
    echo "[start_single_chain_ik_modular_device_service_zmq] ${name} exited during startup, see log: ${log_file}" >&2
    if [[ -f "${log_file}" ]]; then
      sed -n '1,120p' "${log_file}" >&2 || true
    fi
    exit 1
  fi
}

echo "[start_single_chain_ik_modular_device_service_zmq] start runtime"
"${RUNTIME_BIN}" "${RUNTIME_CFG}" >"${RUNTIME_LOG}" 2>&1 &
pids+=("$!")
sleep 1
check_started "${pids[$((${#pids[@]} - 1))]}" "runtime" "${RUNTIME_LOG}"

echo "[start_single_chain_ik_modular_device_service_zmq] start device service"
"${DEVICE_BIN}" "${DEVICE_CFG}" >"${DEVICE_LOG}" 2>&1 &
pids+=("$!")
sleep 1
check_started "${pids[$((${#pids[@]} - 1))]}" "device service" "${DEVICE_LOG}"

echo "[start_single_chain_ik_modular_device_service_zmq] start viewer"
"${VIEWER_BIN}" "${VIEWER_CFG}" >"${VIEWER_LOG}" 2>&1 &
pids+=("$!")
sleep 1
check_started "${pids[$((${#pids[@]} - 1))]}" "viewer" "${VIEWER_LOG}"

echo "[start_single_chain_ik_modular_device_service_zmq] running"
echo "  runtime log: ${RUNTIME_LOG}"
echo "  device  log: ${DEVICE_LOG}"
echo "  viewer  log: ${VIEWER_LOG}"
echo "Press Ctrl+C to stop all."

wait
