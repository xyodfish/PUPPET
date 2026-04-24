#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-${REPO_ROOT}/build}"

RUNTIME_CFG="${1:-${REPO_ROOT}/config/runtime/demo_single_chain_ik_runtime_modular_zmq.yaml}"
SENDER_ENDPOINT="${2:-tcp://127.0.0.1:16661}"
VIEWER_CFG="${3:-${REPO_ROOT}/config/tools/demo_single_chain_ik_visualizer_zmq.yaml}"

RUNTIME_BIN="${BUILD_DIR}/app/cpp/runtime/teleop_runtime_embosa_main"
SENDER_BIN="${BUILD_DIR}/test/demos/cpp/test_zmq_single_chain_ik_sender"
VIEWER_BIN="${BUILD_DIR}/app/cpp/tools/retargeting_mujoco_visualizer_zmq"

for f in "${RUNTIME_BIN}" "${SENDER_BIN}" "${VIEWER_BIN}"; do
  if [[ ! -x "${f}" ]]; then
    echo "[start_single_chain_ik_modular_zmq] missing executable: ${f}" >&2
    echo "Build first: cmake -S . -B build && cmake --build build -j\$(nproc)" >&2
    exit 1
  fi
done

echo "[start_single_chain_ik_modular_zmq] sender payload logic matches embosa_single_chain_ik_sender, transport switched to zmq."

mkdir -p "${REPO_ROOT}/bin/log"
RUNTIME_LOG="${REPO_ROOT}/bin/log/single_chain_ik_runtime_modular_zmq.log"
SENDER_LOG="${REPO_ROOT}/bin/log/single_chain_ik_sender_zmq.log"
VIEWER_LOG="${REPO_ROOT}/bin/log/single_chain_ik_viewer_zmq.log"

pids=()
cleanup() {
  echo "[start_single_chain_ik_modular_zmq] stopping nodes..."
  for pid in "${pids[@]:-}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
  wait || true
}
trap cleanup EXIT INT TERM

echo "[start_single_chain_ik_modular_zmq] start runtime (modular zmq config)"
"${RUNTIME_BIN}" "${RUNTIME_CFG}" >"${RUNTIME_LOG}" 2>&1 &
pids+=("$!")
sleep 1

echo "[start_single_chain_ik_modular_zmq] start sender (test_zmq_single_chain_ik_sender)"
"${SENDER_BIN}" "${SENDER_ENDPOINT}" "puppet_demo/primitive_frame" >"${SENDER_LOG}" 2>&1 &
pids+=("$!")
sleep 1

echo "[start_single_chain_ik_modular_zmq] start viewer (zmq)"
"${VIEWER_BIN}" "${VIEWER_CFG}" >"${VIEWER_LOG}" 2>&1 &
pids+=("$!")

echo "[start_single_chain_ik_modular_zmq] running"
echo "  runtime log: ${RUNTIME_LOG}"
echo "  sender  log: ${SENDER_LOG}"
echo "  viewer  log: ${VIEWER_LOG}"
echo "Press Ctrl+C to stop all."

wait
