#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-${REPO_ROOT}/build}"

RUNTIME_CFG="${1:-${REPO_ROOT}/config/runtime/demo_single_chain_ik_runtime_modular.yaml}"
VIEWER_CFG="${2:-${REPO_ROOT}/config/tools/demo_single_chain_ik_visualizer.yaml}"

RUNTIME_BIN="${BUILD_DIR}/app/cpp/runtime/teleop_runtime_embosa_main"
SENDER_BIN="${BUILD_DIR}/test/demos/cpp/test_embosa_single_chain_ik_sender"
VIEWER_BIN="${BUILD_DIR}/app/cpp/tools/retargeting_mujoco_visualizer"

for f in "${RUNTIME_BIN}" "${SENDER_BIN}" "${VIEWER_BIN}"; do
  if [[ ! -x "${f}" ]]; then
    echo "[start_single_chain_ik_modular_embosa] missing executable: ${f}" >&2
    echo "Build first: cmake -S . -B build && cmake --build build -j\$(nproc)" >&2
    exit 1
  fi
done

mkdir -p "${REPO_ROOT}/bin/log"
RUNTIME_LOG="${REPO_ROOT}/bin/log/single_chain_ik_runtime_modular_embosa.log"
SENDER_LOG="${REPO_ROOT}/bin/log/single_chain_ik_sender_embosa.log"
VIEWER_LOG="${REPO_ROOT}/bin/log/single_chain_ik_viewer_embosa.log"

pids=()
cleanup() {
  echo "[start_single_chain_ik_modular_embosa] stopping nodes..."
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
    echo "[start_single_chain_ik_modular_embosa] ${name} exited during startup, see log: ${log_file}" >&2
    if [[ -f "${log_file}" ]]; then
      sed -n '1,120p' "${log_file}" >&2 || true
    fi
    exit 1
  fi
}

echo "[start_single_chain_ik_modular_embosa] start runtime (modular embosa config)"
"${RUNTIME_BIN}" "${RUNTIME_CFG}" >"${RUNTIME_LOG}" 2>&1 &
pids+=("$!")
sleep 1
check_started "${pids[$((${#pids[@]} - 1))]}" "runtime" "${RUNTIME_LOG}"

echo "[start_single_chain_ik_modular_embosa] start sender (test_embosa_single_chain_ik_sender)"
"${SENDER_BIN}" >"${SENDER_LOG}" 2>&1 &
pids+=("$!")
sleep 1
check_started "${pids[$((${#pids[@]} - 1))]}" "sender" "${SENDER_LOG}"

echo "[start_single_chain_ik_modular_embosa] start viewer"
"${VIEWER_BIN}" "${VIEWER_CFG}" >"${VIEWER_LOG}" 2>&1 &
pids+=("$!")
sleep 1
check_started "${pids[$((${#pids[@]} - 1))]}" "viewer" "${VIEWER_LOG}"

echo "[start_single_chain_ik_modular_embosa] running"
echo "  runtime log: ${RUNTIME_LOG}"
echo "  sender  log: ${SENDER_LOG}"
echo "  viewer  log: ${VIEWER_LOG}"
echo "Press Ctrl+C to stop all."

wait
